#![no_std]
#![no_main]

use core::ops::Range;

use embassy_embedded_hal::{adapter::BlockingAsync, shared_bus::asynch::i2c::I2cDevice};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::{
    Peri, dma, flash, gpio, i2c,
    interrupt::{self, InterruptExt},
    peripherals, pio,
    pio_programs::ws2812,
    pwm, spi, uart, usb,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex as AsyncMutex};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use fixed::FixedU16;
use static_cell::StaticCell;

#[cfg(feature = "defmt")]
use defmt_rtt as _;
use panic_probe as _;

#[macro_use]
extern crate toto_flight;

use toto_flight::{
    config::Config,
    drivers::{battery::ina226::Ina226, imu::lsm6dsv::Lsm6dsv16b},
};

type Outputs =
    toto_flight::control::Outputs<1, 2, pwm::PwmOutput<'static>, pwm::PwmOutput<'static>>;

type I2cBus = i2c::I2c<'static, I2C0, i2c::Async>;
type SharedI2cBus = AsyncMutex<CriticalSectionRawMutex, I2cBus>;
type SharedI2cDevice = I2cDevice<'static, CriticalSectionRawMutex, I2cBus>;

type ImuSpiDevice = ExclusiveDevice<
    spi::Spi<'static, SPI1, spi::Async>,
    gpio::Output<'static>,
    embassy_time::Delay,
>;
type ImuInterruptPin = gpio::Input<'static>;
type ImuSensor = Lsm6dsv16b<ImuSpiDevice>;

type BatterySensor = Ina226<SharedI2cDevice>;

const FLASH_SIZE: u32 = 16 * 1024 * 1024;
const FLASH_STORAGE_RANGE: Range<u32> = FLASH_SIZE - 4 * 0x1000..FLASH_SIZE; // store in the last 4 sectors
type Flash = flash::Flash<'static, FLASH, flash::Blocking, { FLASH_SIZE as usize }>;
type AsyncFlash =
    BlockingAsync<flash::Flash<'static, FLASH, flash::Blocking, { FLASH_SIZE as usize }>>;
type Storage = toto_flight::config::Storage<AsyncFlash>;

type UsbDriver = usb::Driver<'static, USB>;

type PioWs2812Driver = ws2812::PioWs2812<'static, PIO0, 0, ws2812::Grb>;

assign_resources! {
    pwm_outputs: PwmOutputsPeripherals {
        slice_esc_and_left_elevon: PWM_SLICE5,
        slice_right_elevon: PWM_SLICE7,
        esc: PIN_10,
        left_elevon: PIN_11,
        right_elevon: PIN_15,
    },
    imu: ImuPeripherals {
        spi: SPI1,
        clk_pin: PIN_30,
        mosi_pin: PIN_31,
        miso_pin: PIN_28,
        cs_pin: PIN_29,
        tx_dma: DMA_CH0,
        rx_dma: DMA_CH1,
        int_pin: PIN_27,
    },
    receiver: ReceiverPeripherals {
        uart: UART0,
        tx_pin: PIN_0,
        rx_pin: PIN_1,
    },
    gps: GpsPeripherals {
        uart: UART1,
        tx_pin: PIN_4,
        rx_pin: PIN_5,
    },
    i2c_bus: I2cBusPeripherals {
        i2c: I2C0,
        sda_pin: PIN_32,
        scl_pin: PIN_33,
    },
    sd_card: SdCardPeripherals {
        spi: SPI0,
        clk_pin: PIN_34,
        mosi_pin: PIN_35,
        miso_pin: PIN_36,
        cs_pin: PIN_39,
    },
    flash: FlashPeripherals {
        flash: FLASH,
    },
    usb: UsbPeripherals {
        usb: USB,
    },
    led: LedPeripherals {
        pin: PIN_46,
        pio: PIO0,
        dma: DMA_CH2,
    },
}

embassy_rp::bind_interrupts!(pub struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>,
                 dma::InterruptHandler<DMA_CH1>,
                 dma::InterruptHandler<DMA_CH2>;
                 // dma::InterruptHandler<DMA_CH3>,
                 // dma::InterruptHandler<DMA_CH4>,
                 // dma::InterruptHandler<DMA_CH5>,
                 // dma::InterruptHandler<DMA_CH6>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => uart::BufferedInterruptHandler<UART1>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

static REALTIME_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static LOW_PRIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
fn SWI_IRQ_0() {
    unsafe { REALTIME_EXECUTOR.on_interrupt() };
}

#[cortex_m_rt::interrupt]
fn SWI_IRQ_1() {
    unsafe { LOW_PRIO_EXECUTOR.on_interrupt() };
}

#[embassy_executor::main]
async fn main(thread_spawner: Spawner) -> ! {
    _ = thread_spawner;

    let mut cp = cortex_m::Peripherals::take().unwrap();

    let p = embassy_rp::init(embassy_rp::config::Config::default());
    let r = split_resources!(p);

    // init interrupts
    interrupt::SWI_IRQ_0.set_priority(interrupt::Priority::P0); // realtime executor
    interrupt::SWI_IRQ_1.set_priority(interrupt::Priority::P2); // low prio executor

    interrupt::I2C0_IRQ.set_priority(interrupt::Priority::P1);
    interrupt::UART0_IRQ.set_priority(interrupt::Priority::P1);
    interrupt::UART1_IRQ.set_priority(interrupt::Priority::P1);
    interrupt::USBCTRL_IRQ.set_priority(interrupt::Priority::P1);
    interrupt::PIO0_IRQ_0.set_priority(interrupt::Priority::P1);

    let outputs = {
        // Make pwm counter increment once every microsecond
        let pwm_divider = embassy_rp::clocks::clk_sys_freq() / 1_000_000;
        // The frequency of the pwm signal is 50Hz.
        let pwm_top = 20_000;

        let mut config = pwm::Config::default();
        config.top = pwm_top;
        config.divider = FixedU16::from_num(pwm_divider);

        let (Some(throttle_pwm), Some(left_elevon_pwm)) = pwm::Pwm::new_output_ab(
            r.pwm_outputs.slice_esc_and_left_elevon,
            r.pwm_outputs.esc,
            r.pwm_outputs.left_elevon,
            config.clone(),
        )
        .split() else {
            unreachable!()
        };

        let (None, Some(right_elevon_pwm)) = pwm::Pwm::new_output_b(
            r.pwm_outputs.slice_right_elevon,
            r.pwm_outputs.right_elevon,
            config.clone(),
        )
        .split() else {
            unreachable!()
        };

        toto_flight::control::Outputs {
            motors: [throttle_pwm],
            servos: [left_elevon_pwm, right_elevon_pwm],
        }
    };

    let imu_spi_device = {
        let mut config = spi::Config::default();
        config.frequency = 8_000_000;
        let spi = spi::Spi::new(
            r.imu.spi,
            r.imu.clk_pin,
            r.imu.mosi_pin,
            r.imu.miso_pin,
            r.imu.tx_dma,
            r.imu.rx_dma,
            Irqs,
            config,
        );
        let spi_cs = gpio::Output::new(r.imu.cs_pin, gpio::Level::High);
        ExclusiveDevice::new(spi, spi_cs, embassy_time::Delay).unwrap() // NOTE: can't fail
    };
    let imu_interrupt_pin = gpio::Input::new(r.imu.int_pin, gpio::Pull::None);
    let imu_sensor = ImuSensor::new(imu_spi_device, embassy_time::Delay)
        .await
        .unwrap();

    let (receiver_uart_tx, receiver_uart_rx) = {
        static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();

        let mut config = uart::Config::default();
        config.baudrate = 115_200;

        let uart = uart::BufferedUart::new(
            r.receiver.uart,
            r.receiver.tx_pin,
            r.receiver.rx_pin,
            Irqs,
            TX_BUF.init([0; 64]),
            RX_BUF.init([0; 256]),
            config,
        );

        uart.split()
    };

    let gps_uart_rx = {
        static RX_BUF: StaticCell<[u8; 128]> = StaticCell::new();

        let mut config = uart::Config::default();
        config.baudrate = 115_200;

        uart::BufferedUartRx::new(
            r.gps.uart,
            Irqs,
            r.gps.rx_pin,
            RX_BUF.init([0; 128]),
            config,
        )
    };

    let i2c_device = {
        static I2C_BUS: StaticCell<SharedI2cBus> = StaticCell::new();

        let mut config = i2c::Config::default();
        config.frequency = 400_000;
        config.sda_pullup = false;
        config.scl_pullup = false;
        let i2c_bus = i2c::I2c::new_async(
            r.i2c_bus.i2c,
            r.i2c_bus.scl_pin,
            r.i2c_bus.sda_pin,
            Irqs,
            config,
        );

        SharedI2cDevice::new(I2C_BUS.init(AsyncMutex::new(i2c_bus)))
    };
    let battery_sensor = BatterySensor::new(i2c_device.clone()).await.unwrap();

    let flash = BlockingAsync::new(Flash::new_blocking(r.flash.flash));
    let mut storage = toto_flight::config::Storage::new(flash, FLASH_STORAGE_RANGE);

    let usb_driver = usb::Driver::new(r.usb.usb, Irqs);

    let mut led_pio = pio::Pio::new(r.led.pio, Irqs);
    let program = ws2812::PioWs2812Program::new(&mut led_pio.common);
    let led_driver = PioWs2812Driver::new(
        &mut led_pio.common,
        led_pio.sm0,
        r.led.dma,
        Irqs,
        r.led.pin,
        &program,
    );

    static CONFIG: StaticCell<Config> = StaticCell::new();
    let cfg = CONFIG.init(storage.read().await.unwrap());

    let realtime_spawner = REALTIME_EXECUTOR.start(interrupt::SWI_IRQ_0);
    let low_prio_spawner = LOW_PRIO_EXECUTOR.start(interrupt::SWI_IRQ_1);

    info!("setup done. spawning tasks");

    realtime_spawner.spawn(imu_interrupt_listener_task(imu_interrupt_pin).unwrap());
    realtime_spawner.spawn(imu_read_task(cfg, imu_sensor).unwrap());
    realtime_spawner.spawn(control_task(cfg, outputs).unwrap());
    // This is realtime because it is doing blocking stuff in a critical section anyways
    realtime_spawner.spawn(config_task(storage).unwrap());

    low_prio_spawner.spawn(battery_task(battery_sensor).unwrap());
    low_prio_spawner.spawn(fusion_task().unwrap());
    low_prio_spawner.spawn(gps_task(gps_uart_rx).unwrap());
    low_prio_spawner.spawn(imu_calibrate_task(cfg).unwrap());
    low_prio_spawner.spawn(led_task(led_driver).unwrap());
    low_prio_spawner.spawn(receiver_read_task(receiver_uart_rx).unwrap());
    low_prio_spawner.spawn(receiver_telemetry_task(receiver_uart_tx).unwrap());
    low_prio_spawner.spawn(usb_interface_task(usb_driver).unwrap());

    cp.DCB.enable_trace();

    cp.DWT.set_cycle_count(0);
    cp.DWT.set_exception_count(0);
    cp.DWT.enable_cycle_counter();

    // set SEVONPEND flag so that interrupts set the event flag even inside a critical section
    unsafe { cp.SCB.scr.modify(|reg| reg | 0x10) }

    #[cfg(feature = "cpu-usage")]
    let mut cpu_usage_tracker = toto_flight::cpu_usage::Tracker::new();

    loop {
        #[cfg(feature = "cpu-usage")]
        cpu_usage_tracker.on_idle(&mut cp);

        #[cfg(not(feature = "cpu-usage"))]
        cortex_m::asm::wfi();
    }
}

#[embassy_executor::task]
async fn led_task(mut led_driver: PioWs2812Driver) {
    // TODO: Different patterns to show state
    loop {
        led_driver.write_slice(&[(0, 0, 100).into()]).await;
        Timer::after(Duration::from_millis(500)).await;
        led_driver.write_slice(&[(0, 0, 0).into()]).await;
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn battery_task(battery_sensor: BatterySensor) {
    toto_flight::battery::task(battery_sensor).await;
}

#[embassy_executor::task]
async fn control_task(cfg: &'static Config, outputs: Outputs) {
    let control = toto_flight::control::Control::new(cfg);
    control.run(outputs).await;
}

#[embassy_executor::task]
async fn config_task(mut storage: Storage) {
    storage.run().await;
}

#[embassy_executor::task]
async fn fusion_task() {
    toto_flight::fusion::task().await;
}

#[embassy_executor::task]
async fn gps_task(uart_rx: uart::BufferedUartRx) {
    toto_flight::gps::task(uart_rx).await;
}

#[embassy_executor::task]
async fn imu_interrupt_listener_task(pin: ImuInterruptPin) {
    toto_flight::imu::task_interrupt_listener(pin).await;
}

#[embassy_executor::task]
async fn imu_read_task(cfg: &'static Config, imu: ImuSensor) {
    toto_flight::imu::task_read(cfg, imu).await;
}

#[embassy_executor::task]
async fn imu_calibrate_task(cfg: &'static Config) {
    toto_flight::imu::task_calibrate(cfg).await;
}

#[embassy_executor::task]
async fn receiver_read_task(uart_rx: uart::BufferedUartRx) {
    toto_flight::receiver::task_read(uart_rx).await;
}

#[embassy_executor::task]
async fn receiver_telemetry_task(uart_tx: uart::BufferedUartTx) {
    toto_flight::receiver::task_telemetry(uart_tx).await;
}

#[embassy_executor::task]
async fn usb_interface_task(usb_driver: UsbDriver) {
    toto_flight::usb_interface::task(usb_driver).await;
}

#[cfg(feature = "defmt")]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf();
}
