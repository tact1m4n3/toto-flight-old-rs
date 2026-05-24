#![no_std]
#![no_main]
#![warn(clippy::pedantic)]
#![warn(clippy::panic)]
#![recursion_limit = "256"]

use core::ops::Range;

use embassy_embedded_hal::{adapter::BlockingAsync, shared_bus::asynch::i2c::I2cDevice};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::{
    Peri, dma, flash, gpio, i2c,
    interrupt::{self, InterruptExt},
    peripherals, spi, uart, usb,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex as AsyncMutex};
use embassy_time::{Duration, Instant};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_probe as _;

#[macro_use]
extern crate toto_flight;

use toto_flight::drivers::{battery::ina226::Ina226, imu::lsm6dsv::Lsm6dsv16b};

const FLASH_SIZE: usize = 16 * 1024 * 1024;
const FLASH_STORAGE_RANGE: Range<u32> = 15 * 1024 * 1024..16 * 1024 * 1024; // store in the last megabyte

type I2cBus = i2c::I2c<'static, I2C0, i2c::Async>;
type SharedI2cBus = AsyncMutex<CriticalSectionRawMutex, I2cBus>;
type SharedI2cDevice = I2cDevice<'static, CriticalSectionRawMutex, I2cBus>;

type ImuSpiDevice =
    ExclusiveDevice<spi::Spi<'static, SPI1, spi::Async>, gpio::Output<'static>, NoDelay>;
type ImuInterruptPin = gpio::Input<'static>;
type ImuSensor = Lsm6dsv16b<ImuSpiDevice>;

type Flash = flash::Flash<'static, FLASH, flash::Blocking, FLASH_SIZE>;
type AsyncFlash = BlockingAsync<flash::Flash<'static, FLASH, flash::Blocking, FLASH_SIZE>>;
type Storage = toto_flight::config::Storage<AsyncFlash>;

type BatterySensor = Ina226<SharedI2cDevice>;

assign_resources! {
    led: LedPeripherals {
        pin: PIN_45,
    },
    usb: UsbPeripherals {
        usb: USB,
    },
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
    i2c_bus: I2cBusPeripherals {
        i2c: I2C0,
        sda_pin: PIN_32,
        scl_pin: PIN_33,
    },
    receiver: ReceiverPeripherals {
        uart: UART0,
        tx_pin: PIN_0,
        rx_pin: PIN_1,
        tx_dma: DMA_CH2,
    },
    gps: GpsPeripherals {
        uart: UART1,
        tx_pin: PIN_4,
        rx_pin: PIN_5,
        pps_pin: PIN_14,
        tx_dma: DMA_CH3,
    },
    sd_card: SdCardPeripherals {
        spi: SPI0,
        clk_pin: PIN_34,
        mosi_pin: PIN_35,
        miso_pin: PIN_36,
        tx_dma: DMA_CH4,
        rx_dma: DMA_CH5,
        cs_pin: PIN_39,
    },
    flash: FlashPeripherals {
        flash: FLASH,
    },
}

embassy_rp::bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => uart::BufferedInterruptHandler<UART1>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>,
                 dma::InterruptHandler<DMA_CH1>,
                 dma::InterruptHandler<DMA_CH2>,
                 dma::InterruptHandler<DMA_CH3>,
                 dma::InterruptHandler<DMA_CH4>,
                 dma::InterruptHandler<DMA_CH5>,
                 dma::InterruptHandler<DMA_CH6>;
});

static REALTIME_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static UART_READ_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static NAV_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static LOW_PRIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
fn SWI_IRQ_0() {
    unsafe { REALTIME_EXECUTOR.on_interrupt() };
}

#[cortex_m_rt::interrupt]
fn SWI_IRQ_1() {
    unsafe { UART_READ_EXECUTOR.on_interrupt() };
}

#[cortex_m_rt::interrupt]
fn SWI_IRQ_2() {
    unsafe { NAV_EXECUTOR.on_interrupt() };
}

#[cortex_m_rt::interrupt]
fn SWI_IRQ_3() {
    unsafe { LOW_PRIO_EXECUTOR.on_interrupt() };
}

#[embassy_executor::main]
async fn main(thread_spawner: Spawner) -> ! {
    _ = thread_spawner;

    let mut cp = cortex_m::Peripherals::take().unwrap();

    let p = embassy_rp::init(embassy_rp::config::Config::default());
    let r = split_resources!(p);

    // init interrupts
    interrupt::SWI_IRQ_0.set_priority(interrupt::Priority::P15); // realtime executor
    interrupt::SWI_IRQ_1.set_priority(interrupt::Priority::P2); // uart read executor
    interrupt::SWI_IRQ_2.set_priority(interrupt::Priority::P1); // nav executor
    interrupt::SWI_IRQ_3.set_priority(interrupt::Priority::P0); // low prio executor

    // embassy interrupts
    interrupt::USBCTRL_IRQ.set_priority(interrupt::Priority::P3);
    interrupt::I2C0_IRQ.set_priority(interrupt::Priority::P3);
    interrupt::UART0_IRQ.set_priority(interrupt::Priority::P3);
    interrupt::UART1_IRQ.set_priority(interrupt::Priority::P3);

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
        ExclusiveDevice::new(spi, spi_cs, NoDelay).unwrap() // NOTE: can't fail
    };

    let imu_interrupt_pin = gpio::Input::new(r.imu.int_pin, gpio::Pull::None);

    let (receiver_uart_tx, receiver_uart_rx) = {
        static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
        static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();

        let mut config = uart::Config::default();
        config.baudrate = 115_200;

        let uart = uart::BufferedUart::new(
            r.receiver.uart,
            r.receiver.tx_pin,
            r.receiver.rx_pin,
            Irqs,
            TX_BUF.init([0; 256]),
            RX_BUF.init([0; 256]),
            config,
        );

        uart.split()
    };

    let imu_sensor = ImuSensor::new(imu_spi_device, embassy_time::Delay)
        .await
        .unwrap();
    let battery_sensor = BatterySensor::new(i2c_device.clone()).await.unwrap();

    let flash = BlockingAsync::new(Flash::new_blocking(r.flash.flash));
    let storage = toto_flight::config::Storage::new(flash, FLASH_STORAGE_RANGE)
        .await
        .unwrap();

    let realtime_spawner = REALTIME_EXECUTOR.start(interrupt::SWI_IRQ_0);
    let uart_read_spawner = UART_READ_EXECUTOR.start(interrupt::SWI_IRQ_1);
    let nav_spawner = NAV_EXECUTOR.start(interrupt::SWI_IRQ_2);
    let low_prio_spawner = LOW_PRIO_EXECUTOR.start(interrupt::SWI_IRQ_3);

    realtime_spawner.spawn(imu_interrupt_listener_task(imu_interrupt_pin).unwrap());
    realtime_spawner.spawn(imu_read_task(imu_sensor).unwrap());

    uart_read_spawner.spawn(receiver_read_task(receiver_uart_rx).unwrap());

    low_prio_spawner.spawn(battery_task(battery_sensor).unwrap());
    low_prio_spawner.spawn(receiver_telemetry_task(receiver_uart_tx).unwrap());
    low_prio_spawner.spawn(config_task(storage).unwrap());

    cp.DCB.enable_trace();

    cp.DWT.set_cycle_count(0);
    cp.DWT.set_exception_count(0);
    cp.DWT.enable_cycle_counter();

    // set SEVONPEND flag so that interrupts set the event flag even inside a critical section
    unsafe { cp.SCB.scr.modify(|reg| reg | 0x10) }

    let mut led = gpio::Output::new(r.led.pin, gpio::Level::Low);

    let mut last_tick_ticks: u32 = 0;
    let mut idle_ticks: u32 = 0;
    let mut last_tick_instant: Instant = Instant::now();
    loop {
        // TODO: Add feature flag to disable cpu usage monitoring since it adds some overhead
        let mut before: u32 = 0;
        let mut after: u32 = 0;
        critical_section::with(|_| {
            before = cp.DWT.cyccnt.read();
            cortex_m::asm::wfe();
            after = cp.DWT.cyccnt.read();
        });
        idle_ticks += after.wrapping_sub(before);

        let now_instant = Instant::now();
        if now_instant - last_tick_instant > Duration::from_secs(1) {
            let ticks_now = cp.DWT.cyccnt.read();
            let tick_diff = ticks_now.wrapping_sub(last_tick_ticks);
            let cpu_usage =
                100u64.saturating_sub(u64::from(idle_ticks) * 100 / u64::from(tick_diff));

            defmt::info!("cpu usage: {}%", cpu_usage);

            led.toggle();

            last_tick_instant = now_instant;
            last_tick_ticks = ticks_now;
            idle_ticks = 0;
        }
    }
}

#[embassy_executor::task]
async fn imu_interrupt_listener_task(pin: ImuInterruptPin) {
    toto_flight::imu::task_interrupt_listener(pin).await;
}

#[embassy_executor::task]
async fn imu_read_task(imu: ImuSensor) {
    toto_flight::imu::task_read(imu).await;
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
async fn battery_task(battery_sensor: BatterySensor) {
    toto_flight::battery::task(battery_sensor).await;
}

#[embassy_executor::task]
async fn config_task(mut storage: Storage) {
    storage.run().await;
}

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf();
}
