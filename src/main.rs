#![no_std]
#![no_main]
#![recursion_limit = "256"]
// NOTE: used by usb_interface
#![feature(str_split_whitespace_remainder)]

mod black_box;
mod drivers;
mod nav_loop;
mod persistent_config;
mod pid_loop;
mod receiver;
mod sensors;
mod usb_interface;
mod utils;

use assign_resources::assign_resources;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, InterruptExecutor, SendSpawner};
use embassy_rp::{
    Peri, dma, gpio, i2c,
    interrupt::{self, InterruptExt},
    peripherals, uart, usb,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex as AsyncMutex};
use embassy_time::{Duration, Instant};
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_probe as _;

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
        dma: DMA_CH6,
    },
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = embassy_rp::init(Default::default());

    let r = split_resources!(p);

    interrupt::SWI_IRQ_0.set_priority(interrupt::Priority::P2); // realtime
    interrupt::SWI_IRQ_1.set_priority(interrupt::Priority::P0); // default

    // interrupts used by realtime tasks
    interrupt::TIMER0_IRQ_0.set_priority(interrupt::Priority::P3);
    interrupt::DMA_IRQ_0.set_priority(interrupt::Priority::P3);
    interrupt::IO_IRQ_BANK0.set_priority(interrupt::Priority::P3);

    // uart reader interrupts
    interrupt::UART0_IRQ.set_priority(interrupt::Priority::P1);
    interrupt::UART1_IRQ.set_priority(interrupt::Priority::P1);

    // interrupts used by default tasks
    interrupt::USBCTRL_IRQ.set_priority(interrupt::Priority::P1);
    interrupt::I2C0_IRQ.set_priority(interrupt::Priority::P1);

    let spawner = SystemSpawner {
        realtime: REALTIME_EXECUTOR.start(interrupt::SWI_IRQ_0),
        default: DEFAULT_EXECUTOR.start(interrupt::SWI_IRQ_1),
    };

    static IDLE_EXECUTOR: StaticCell<Executor> = StaticCell::new();
    IDLE_EXECUTOR
        .init(Executor::new())
        .run(|idle_spawner| idle_spawner.spawn(idle(spawner, cp, r).unwrap()));
}

#[embassy_executor::task]
pub async fn idle(spawner: SystemSpawner, mut cp: cortex_m::Peripherals, r: AssignedResources) {
    persistent_config::init(r.flash).await;

    let i2c_device = init_i2c(r.i2c_bus);

    pid_loop::init(spawner, r.pwm_outputs);
    nav_loop::init(spawner);

    receiver::init(spawner, r.receiver);

    sensors::imu::init(spawner, r.imu).await.unwrap();
    sensors::gps::init(spawner, r.gps);
    sensors::multimeter::init(spawner, i2c_device.clone())
        .await
        .unwrap();
    sensors::fusion::init(spawner);

    usb_interface::init(spawner, r.usb);

    let mut storage = match black_box::Storage::new(r.sd_card) {
        Ok(storage) => Some(storage),
        Err(err) => {
            defmt::warn!("failed to initialize black box logging: {}", err);
            None
        }
    };

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
        if let Some(storage) = storage.as_mut() {
            match storage.poll() {
                Ok(()) => (),
                Err(err) => {
                    defmt::warn!("black box logging error: {}", err);
                }
            }
        }

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
            let cpu_usage = 100u64.saturating_sub(idle_ticks as u64 * 100 / tick_diff as u64);

            defmt::info!("cpu usage: {}%", cpu_usage);

            led.toggle();

            last_tick_instant = now_instant;
            last_tick_ticks = ticks_now;
            idle_ticks = 0;
        }
    }
}

type I2cBus = i2c::I2c<'static, I2C0, i2c::Async>;
type SharedI2cBus = AsyncMutex<CriticalSectionRawMutex, I2cBus>;
pub type SharedI2cDevice = I2cDevice<'static, CriticalSectionRawMutex, I2cBus>;

fn init_i2c(p: I2cBusPeripherals) -> SharedI2cDevice {
    let mut config = i2c::Config::default();
    config.frequency = 400_000;
    config.sda_pullup = false;
    config.scl_pullup = false;
    let i2c_bus = i2c::I2c::new_async(p.i2c, p.scl_pin, p.sda_pin, Irqs, config);
    static I2C_BUS: StaticCell<SharedI2cBus> = StaticCell::new();
    I2cDevice::new(I2C_BUS.init(AsyncMutex::new(i2c_bus)))
}

#[derive(Clone, Copy)]
pub struct SystemSpawner {
    pub realtime: SendSpawner,
    pub default: SendSpawner,
}

static REALTIME_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static DEFAULT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
fn SWI_IRQ_0() {
    unsafe { REALTIME_EXECUTOR.on_interrupt() };
}

#[cortex_m_rt::interrupt]
fn SWI_IRQ_1() {
    unsafe { DEFAULT_EXECUTOR.on_interrupt() };
}

embassy_rp::bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    UART1_IRQ => uart::BufferedInterruptHandler<UART1>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>,
                 dma::InterruptHandler<DMA_CH1>,
                 dma::InterruptHandler<DMA_CH2>,
                 dma::InterruptHandler<DMA_CH3>,
                 dma::InterruptHandler<DMA_CH4>,
                 dma::InterruptHandler<DMA_CH5>,
                 dma::InterruptHandler<DMA_CH6>;
});

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf();
}

// let world_up = Vector3::new(0.0, 0.0, 1.0);
// let body_up = attitude.inverse_transform_vector(&world_up);
//
// let target_up = UnitQuaternion::from_euler_angles(roll_angle, pitch_angle, 0.0)
//     .inverse_transform_vector(&world_up);
//
// let angle_error = {
//     let cross = body_up.cross(&target_up);
//     let dot = body_up.dot(&target_up).clamp(-1.0, 1.0);
//
//     let angle = libm::atan2f(cross.norm(), dot);
//
//     let angle_error = if cross.norm() > 1e-6 {
//         cross.normalize() * angle
//     } else {
//         Vector3::zeros()
//     };
//
//     angle_error
// };
//
// let target_rates = Vector3::new(
//     angle_error.x * ANGLE_ROLL_KP,
//     angle_error.y * ANGLE_PITCH_KP,
//     yaw_rate,
// );
//
// (
//     throttle,
//     rate_full_pid_controller.update(&target_rates, &gyro, sensors::IMU_FREQ_DT),
// )
