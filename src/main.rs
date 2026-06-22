#![no_std]
#![no_main]

use core::ops::Range;

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
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::sdcard::spi::AcquireOpts;
use fixed::FixedU16;
use static_cell::StaticCell;

#[cfg(feature = "defmt")]
use defmt_rtt as _;
use panic_probe as _;

#[macro_use]
extern crate toto_flight;

use toto_flight::drivers::{battery::ina226::Ina226, imu::lsm6dsv::Lsm6dsv16b, motor::dshot_pio};

type Outputs = toto_flight::control::Outputs<
    1,
    2,
    dshot_pio::DshotPio<'static, PIO0>,
    [pwm::PwmOutput<'static>; 2],
>;

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

    let sd_card_spi_device = {
        let mut config = spi::Config::default();
        config.frequency = 100_000;
        let spi = spi::Spi::new_blocking(
            r.sd_card.spi,
            r.sd_card.clk_pin,
            r.sd_card.mosi_pin,
            r.sd_card.miso_pin,
            config,
        );
        let spi_cs = gpio::Output::new(r.sd_card.cs_pin, gpio::Level::High);
        ExclusiveDevice::new_no_delay(spi, spi_cs).unwrap() // NOTE: can't fail
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

    info!("setup done. spawning tasks");

    realtime_spawner.spawn(task_reboot().unwrap());

    // This is realtime because it is doing blocking stuff in a critical section anyways
    realtime_spawner.spawn(task_config(flash, FLASH_STORAGE_RANGE).unwrap());

    realtime_spawner.spawn(task_control_inner_loop(outputs).unwrap());
    realtime_spawner.spawn(task_imu_interrupt_listener(imu_interrupt_pin).unwrap());
    realtime_spawner.spawn(task_imu_read(imu_sensor).unwrap());

    low_prio_spawner.spawn(task_battery(battery_sensor).unwrap());
    low_prio_spawner.spawn(task_command_guardian().unwrap());
    low_prio_spawner.spawn(task_control_outer_loop().unwrap());
    low_prio_spawner.spawn(task_gps(gps_uart_rx).unwrap());
    low_prio_spawner.spawn(task_sensor_fusion().unwrap());
    low_prio_spawner.spawn(task_imu_calibrate().unwrap());
    low_prio_spawner.spawn(task_receiver_channels_manager().unwrap());
    low_prio_spawner.spawn(task_receiver_read_crsf(receiver_uart_rx).unwrap());
    low_prio_spawner.spawn(task_receiver_telemetry_crsf(receiver_uart_tx).unwrap());
    // #[cfg(feature = "defmt")]
    // low_prio_spawner.spawn(task_sensors_debug_log().unwrap());
    low_prio_spawner.spawn(task_status_led(led_driver).unwrap());
    low_prio_spawner.spawn(task_usb_interface(usb_driver).unwrap());
    low_prio_spawner.spawn(task_vtx_msp_read(vtx_uart_rx).unwrap());
    low_prio_spawner.spawn(task_vtx_msp_write(vtx_uart_tx).unwrap());

    let mut maybe_black_box = {
        let sd_card = embedded_sdmmc::SdCard::new_with_options(
            sd_card_spi_device,
            Delay,
            AcquireOpts {
                use_crc: true,
                acquire_retries: 1,
            },
        );

        // this also initializes the sdcard
        if let Ok(_sd_card_size) = sd_card.num_bytes() {
            info!("sd card size: {}MB", _sd_card_size / 1024 / 1024);

            // up the frequency after initialization
            let mut config = spi::Config::default();
            config.frequency = 12_000_000;
            sd_card.spi(|dev| dev.bus_mut().set_config(&config));

            Some(sd_card)
        } else {
            None
        }
    }
    .and_then(|sd_card| toto_flight::black_box::Storage::new(sd_card).ok());

    #[cfg(feature = "cpu-usage")]
    let mut cpu_usage_tracker = toto_flight::cpu_usage::Tracker::new(&mut _cp);

    loop {
        maybe_black_box.take_if(|black_box| black_box.poll().is_err());

        #[cfg(feature = "cpu-usage")]
        cpu_usage_tracker.on_idle(&mut _cp);

        #[cfg(not(feature = "cpu-usage"))]
        cortex_m::asm::wfi();
    }
}

#[embassy_executor::task]
async fn task_reboot() {
    loop {
        toto_flight::REBOOT.wait().await;
        Timer::after(Duration::from_millis(100)).await;
        if let Ok(_guard) = toto_flight::control::ArmBlocker::new() {
            info!("rebooting...");
            cortex_m::peripheral::SCB::sys_reset();
        } else {
            warn!("reboot requested but system is armed, ignoring");
        }
    }
}

#[embassy_executor::task]
async fn task_config(flash: AsyncFlash, range: Range<u32>) {
    toto_flight::config::task(flash, range).await;
}

#[embassy_executor::task]
async fn task_command_guardian() {
    toto_flight::control::task_command_guardian().await;
}

#[embassy_executor::task]
async fn task_control_inner_loop(outputs: Outputs) {
    toto_flight::control::task_inner_loop(outputs).await;
}

#[embassy_executor::task]
async fn task_control_outer_loop() {
    toto_flight::control::task_outer_loop().await;
}

#[embassy_executor::task]
async fn task_battery(battery_sensor: BatterySensor) {
    toto_flight::sensors::battery::task(battery_sensor).await;
}

#[embassy_executor::task]
async fn task_gps(uart_rx: uart::BufferedUartRx) {
    toto_flight::sensors::gps::task(uart_rx).await;
}

#[embassy_executor::task]
async fn task_imu_interrupt_listener(pin: ImuInterruptPin) {
    toto_flight::sensors::imu::task_interrupt_listener(pin).await;
}

#[embassy_executor::task]
async fn task_imu_read(imu: ImuSensor) {
    toto_flight::sensors::imu::task_read(imu).await;
}

#[embassy_executor::task]
async fn task_imu_calibrate() {
    toto_flight::sensors::imu::task_calibrate().await;
}

#[embassy_executor::task]
async fn task_sensor_fusion() {
    toto_flight::sensors::fusion::task().await;
}

#[embassy_executor::task]
async fn task_receiver_read_crsf(uart_rx: uart::BufferedUartRx) {
    toto_flight::receiver::task_read_crsf(uart_rx).await;
}

#[embassy_executor::task]
async fn task_receiver_channels_manager() {
    toto_flight::receiver::task_channels_manager().await;
}

#[embassy_executor::task]
async fn task_receiver_telemetry_crsf(uart_tx: uart::BufferedUartTx) {
    toto_flight::receiver::task_telemetry(uart_tx).await;
}

#[embassy_executor::task]
async fn task_vtx_msp_read(uart_rx: VtxUartRx) {
    toto_flight::msp::task_read(uart_rx).await;
}

#[embassy_executor::task]
async fn task_vtx_msp_write(uart_tx: VtxUartTx) {
    toto_flight::msp::task_write(uart_tx).await;
}

#[embassy_executor::task]
async fn task_usb_interface(usb_driver: UsbDriver) {
    toto_flight::usb_interface::task(usb_driver).await;
}

#[embassy_executor::task]
async fn task_status_led(led_driver: PioWs2812Driver) {
    toto_flight::status_led::task(led_driver).await;
}

#[cfg(feature = "defmt")]
#[embassy_executor::task]
async fn task_sensors_debug_log() {
    toto_flight::sensors::debug_log::task().await;
}

#[cfg(feature = "defmt")]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf();
}
