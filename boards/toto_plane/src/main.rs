#![no_std]
#![no_main]

use core::ops::Range;

use block_device_adapters::BufStream;
use embassy_embedded_hal::{adapter::BlockingAsync, shared_bus::asynch::i2c::I2cDevice};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::{
    Peri, dma, flash, gpio, i2c,
    interrupt::{self, InterruptExt},
    peripherals,
    peripherals::*,
    pio,
    pio_programs::{uart as pio_uart, ws2812},
    pwm, spi, uart, usb,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex as AsyncMutex};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use fixed::FixedU16;
use static_cell::StaticCell;

use fc::drivers::{battery::ina226::Ina226, imu::lsm6dsv::Lsm6dsv16b, motor::dshot_pio};

#[cfg(feature = "debug")]
use defmt_rtt as _;
use panic_probe as _;

#[macro_use]
extern crate fc;

#[cfg(feature = "debug")]
mod cpu_usage;

type Outputs =
    fc::control::Outputs<1, 2, dshot_pio::DshotPio<'static, PIO0>, [pwm::PwmOutput<'static>; 2]>;

type VtxUartTx = pio_uart::PioUartTx<'static, PIO1, 0>;
type VtxUartRx = pio_uart::PioUartRx<'static, PIO1, 1>;

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

type UsbDriver = usb::Driver<'static, USB>;

type PioWs2812Driver = ws2812::PioWs2812<'static, PIO2, 0, ws2812::Grb>;

assign_resources! {
    esc: EscOutputPeripherals {
        pio: PIO0,
        signal_pin: PIN_6,
        telemetry_pin: PIN_16,
    },
    servos: ServoOutputsPeripherals {
        left_elevon_slice: PWM_SLICE5,
        right_elevon_slice: PWM_SLICE7,
        left_elevon_pin: PIN_11,
        right_elevon_pin: PIN_15,
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
    vtx: VtxPeripherals {
        pio: PIO1,
        tx_pin: PIN_12,
        rx_pin: PIN_13,
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
        tx_dma: DMA_CH3,
        rx_dma: DMA_CH4,
    },
    flash: FlashPeripherals {
        flash: FLASH,
    },
    usb: UsbPeripherals {
        usb: USB,
    },
    led: LedPeripherals {
        pin: PIN_46,
        pio: PIO2,
        dma: DMA_CH2,
    },
}

embassy_rp::bind_interrupts!(pub struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>,
                 dma::InterruptHandler<DMA_CH1>,
                 dma::InterruptHandler<DMA_CH2>,
                 dma::InterruptHandler<DMA_CH3>,
                 dma::InterruptHandler<DMA_CH4>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => uart::BufferedInterruptHandler<UART1>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
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

    let mut _cp = cortex_m::Peripherals::take().unwrap();

    let p = embassy_rp::init(embassy_rp::config::Config::default());
    let r = split_resources!(p);

    // init interrupts
    interrupt::SWI_IRQ_0.set_priority(interrupt::Priority::P1); // realtime executor
    interrupt::SWI_IRQ_1.set_priority(interrupt::Priority::P3); // low prio executor

    // used by the inner control loop
    interrupt::PIO0_IRQ_0.set_priority(interrupt::Priority::P0);

    interrupt::I2C0_IRQ.set_priority(interrupt::Priority::P2);
    interrupt::UART0_IRQ.set_priority(interrupt::Priority::P2);
    interrupt::UART1_IRQ.set_priority(interrupt::Priority::P2);
    interrupt::USBCTRL_IRQ.set_priority(interrupt::Priority::P2);
    interrupt::PIO1_IRQ_0.set_priority(interrupt::Priority::P2);
    interrupt::PIO2_IRQ_0.set_priority(interrupt::Priority::P2);

    let esc_dshot = dshot_pio::DshotPio::new(
        r.esc.pio,
        Irqs,
        r.esc.signal_pin,
        dshot_pio::Speed::Dshot300,
    );

    let servos = {
        // Make pwm counter increment once every microsecond
        let pwm_divider = embassy_rp::clocks::clk_sys_freq() / 1_000_000;
        // The frequency of the pwm signal is 50Hz.
        let pwm_top = 20_000;

        let mut config = pwm::Config::default();
        config.top = pwm_top;
        config.divider = FixedU16::from_num(pwm_divider);

        let (None, Some(left_elevon_pwm)) = pwm::Pwm::new_output_b(
            r.servos.left_elevon_slice,
            r.servos.left_elevon_pin,
            config.clone(),
        )
        .split() else {
            unreachable!()
        };

        let (None, Some(right_elevon_pwm)) = pwm::Pwm::new_output_b(
            r.servos.right_elevon_slice,
            r.servos.right_elevon_pin,
            config.clone(),
        )
        .split() else {
            unreachable!()
        };

        [left_elevon_pwm, right_elevon_pwm]
    };

    let outputs = Outputs {
        motors: esc_dshot,
        servos,
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

    let (vtx_uart_tx, vtx_uart_rx) = {
        let baud_rate = 115_200;

        let mut pio = pio::Pio::new(r.vtx.pio, Irqs);

        let tx_program = pio_uart::PioUartTxProgram::new(&mut pio.common);
        let rx_program = pio_uart::PioUartRxProgram::new(&mut pio.common);

        let tx = pio_uart::PioUartTx::new(
            baud_rate,
            &mut pio.common,
            pio.sm0,
            r.vtx.tx_pin,
            &tx_program,
        );
        let rx = pio_uart::PioUartRx::new(
            baud_rate,
            &mut pio.common,
            pio.sm1,
            r.vtx.rx_pin,
            &rx_program,
        );

        (tx, rx)
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

    let sd_card_buf_stream = {
        let mut config = spi::Config::default();
        config.frequency = 100_000;

        let mut spi = spi::Spi::new(
            r.sd_card.spi,
            r.sd_card.clk_pin,
            r.sd_card.mosi_pin,
            r.sd_card.miso_pin,
            r.sd_card.tx_dma,
            r.sd_card.rx_dma,
            Irqs,
            config,
        );

        let mut cs = gpio::Output::new(r.sd_card.cs_pin, gpio::Level::High);

        sdspi::sd_init(&mut spi, &mut cs).await.unwrap();

        let spi_device = ExclusiveDevice::new(spi, cs, embassy_time::Delay).unwrap();

        let mut sd = sdspi::SdSpi::<_, _, aligned::A1>::new(spi_device, embassy_time::Delay);

        sd.init().await.unwrap();
        let size = sd.size().await.unwrap();
        info!("sd card size: {}MB", (size / 1024 / 1024) as u64);

        let mut config = spi::Config::default();
        config.frequency = 12_000_000;
        sd.spi().bus_mut().set_config(&config);

        BufStream::<_, 512>::new(sd)
    };

    let battery_sensor = BatterySensor::new(i2c_device.clone()).await.unwrap();

    let flash = BlockingAsync::new(Flash::new_blocking(r.flash.flash));

    let usb_driver = usb::Driver::new(r.usb.usb, Irqs);

    let mut led_pio = pio::Pio::new(r.led.pio, Irqs);
    let ws2812_program = ws2812::PioWs2812Program::new(&mut led_pio.common);
    let led_driver = PioWs2812Driver::new(
        &mut led_pio.common,
        led_pio.sm0,
        r.led.dma,
        Irqs,
        r.led.pin,
        &ws2812_program,
    );

    let realtime_spawner = REALTIME_EXECUTOR.start(interrupt::SWI_IRQ_0);
    let low_prio_spawner = LOW_PRIO_EXECUTOR.start(interrupt::SWI_IRQ_1);

    realtime_spawner.spawn(task_reboot().unwrap());

    // This is realtime because it is doing blocking stuff in a critical section anyways
    realtime_spawner.spawn(task_config(flash, FLASH_STORAGE_RANGE).unwrap());

    realtime_spawner.spawn(task_control_inner_loop(outputs).unwrap());
    realtime_spawner.spawn(task_imu_interrupt_listener(imu_interrupt_pin).unwrap());
    realtime_spawner.spawn(task_imu_read(imu_sensor).unwrap());

    low_prio_spawner.spawn(task_battery(battery_sensor).unwrap());
    low_prio_spawner.spawn(task_control_outer_loop().unwrap());
    low_prio_spawner.spawn(task_gps(gps_uart_rx).unwrap());
    low_prio_spawner.spawn(task_sensor_fusion().unwrap());
    low_prio_spawner.spawn(task_imu_calibrate().unwrap());
    low_prio_spawner.spawn(task_receiver_channels_manager().unwrap());
    low_prio_spawner.spawn(task_receiver_read_crsf(receiver_uart_rx).unwrap());
    low_prio_spawner.spawn(task_receiver_telemetry_crsf(receiver_uart_tx).unwrap());
    low_prio_spawner.spawn(task_status_led(led_driver).unwrap());
    low_prio_spawner.spawn(task_usb_interface(usb_driver).unwrap());
    low_prio_spawner.spawn(task_vtx_msp_read(vtx_uart_rx).unwrap());
    low_prio_spawner.spawn(task_vtx_msp_write(vtx_uart_tx).unwrap());

    // TODO: Spawn black box task with sd_card_buf_stream once fc::black_box is refactored
    // to use async fast-fat32. The sd_card_buf_stream must be kept alive in main's scope
    // until then.
    let _sd_card_buf_stream = sd_card_buf_stream;

    #[cfg(feature = "debug")]
    let mut cpu_usage_tracker = cpu_usage::Tracker::new(&mut _cp);

    loop {
        #[cfg(feature = "debug")]
        cpu_usage_tracker.on_idle(&mut _cp);

        #[cfg(not(feature = "debug"))]
        cortex_m::asm::wfi();
    }
}

#[embassy_executor::task]
async fn task_reboot() {
    loop {
        fc::REBOOT.wait().await;
        Timer::after(Duration::from_millis(100)).await;
        if !fc::control::status().arm_state.is_armed() {
            warn!("reboot requested but system is armed, ignoring");
            continue;
        }
        info!("rebooting...");
        cortex_m::peripheral::SCB::sys_reset();
    }
}

#[embassy_executor::task]
async fn task_config(flash: AsyncFlash, range: Range<u32>) {
    fc::config::task(flash, range).await;
}

#[embassy_executor::task]
async fn task_control_inner_loop(outputs: Outputs) {
    fc::control::task_inner_loop(outputs).await;
}

#[embassy_executor::task]
async fn task_control_outer_loop() {
    fc::control::task_outer_loop().await;
}

#[embassy_executor::task]
async fn task_battery(battery_sensor: BatterySensor) {
    fc::sensors::battery::task(battery_sensor).await;
}

#[embassy_executor::task]
async fn task_gps(uart_rx: uart::BufferedUartRx) {
    fc::sensors::gps::task(uart_rx).await;
}

#[embassy_executor::task]
async fn task_imu_interrupt_listener(pin: ImuInterruptPin) {
    fc::sensors::imu::task_interrupt_listener(pin).await;
}

#[embassy_executor::task]
async fn task_imu_read(imu: ImuSensor) {
    fc::sensors::imu::task_read(imu).await;
}

#[embassy_executor::task]
async fn task_imu_calibrate() {
    fc::sensors::imu::task_calibrate().await;
}

#[embassy_executor::task]
async fn task_sensor_fusion() {
    fc::sensors::fusion::task().await;
}

#[embassy_executor::task]
async fn task_receiver_read_crsf(uart_rx: uart::BufferedUartRx) {
    fc::receiver::task_read_crsf(uart_rx).await;
}

#[embassy_executor::task]
async fn task_receiver_channels_manager() {
    fc::receiver::task_channels_manager().await;
}

#[embassy_executor::task]
async fn task_receiver_telemetry_crsf(uart_tx: uart::BufferedUartTx) {
    fc::receiver::task_telemetry(uart_tx).await;
}

#[embassy_executor::task]
async fn task_vtx_msp_read(uart_rx: VtxUartRx) {
    fc::msp::task_read(uart_rx).await;
}

#[embassy_executor::task]
async fn task_vtx_msp_write(uart_tx: VtxUartTx) {
    fc::msp::task_write(uart_tx).await;
}

#[embassy_executor::task]
async fn task_usb_interface(usb_driver: UsbDriver) {
    fc::usb_interface::task(usb_driver).await;
}

#[embassy_executor::task]
async fn task_status_led(led_driver: PioWs2812Driver) {
    fc::status_led::task(led_driver).await;
}

#[cfg(feature = "debug")]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf();
}
