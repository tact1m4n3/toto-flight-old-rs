#![no_std]
#![no_main]

mod black_box;
mod control;
mod drivers;
mod motor_output;
mod persistent_config;
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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Instant;
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_probe as _;

assign_resources! {
    multicore: MulticorePeripherals {
        core1: CORE1,
    },
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
    },
    gps: GpsPeripherals {
        uart: UART1,
        tx_pin: PIN_4,
        rx_pin: PIN_5,
        pps_pin: PIN_14,
    },
    sd_card: SdCardPeripherals {
        spi: SPI0,
        clk_pin: PIN_34,
        mosi_pin: PIN_35,
        miso_pin: PIN_36,
        tx_dma: DMA_CH2,
        rx_dma: DMA_CH3,
        cs_pin: PIN_39,
    },
    flash: FlashPeripherals {
        flash: FLASH,
        dma: DMA_CH4,
    },
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    let r = split_resources!(p);

    interrupt::SWI_IRQ_0.set_priority(interrupt::Priority::P3); // realtime
    interrupt::SWI_IRQ_1.set_priority(interrupt::Priority::P2); // high
    interrupt::SWI_IRQ_2.set_priority(interrupt::Priority::P0); // low

    interrupt::TIMER0_IRQ_0.set_priority(interrupt::Priority::P3);
    interrupt::DMA_IRQ_0.set_priority(interrupt::Priority::P3);
    interrupt::IO_IRQ_BANK0.set_priority(interrupt::Priority::P3);

    interrupt::UART0_IRQ.set_priority(interrupt::Priority::P2);
    interrupt::UART1_IRQ.set_priority(interrupt::Priority::P2);
    interrupt::USBCTRL_IRQ.set_priority(interrupt::Priority::P1);

    let spawner = SystemSpawner {
        realtime: REALTIME_EXECUTOR.start(interrupt::SWI_IRQ_0),
        high_prio: HIGH_PRIO_EXECUTOR.start(interrupt::SWI_IRQ_1),
        low_prio: LOW_PRIO_EXECUTOR.start(interrupt::SWI_IRQ_2),
    };

    static IDLE_EXECUTOR: StaticCell<Executor> = StaticCell::new();
    IDLE_EXECUTOR.init(Executor::new()).run(|idle_spawner| {
        idle_spawner.spawn(idle_task(spawner, r).unwrap());
    });
}

#[embassy_executor::task]
pub async fn idle_task(spawner: SystemSpawner, r: AssignedResources) {
    persistent_config::init(r.flash).await;

    let i2c_device = init_i2c(r.i2c_bus);

    control::init(spawner);

    receiver::init(spawner, r.receiver);

    motor_output::init(spawner, r.pwm_outputs);

    sensors::imu::init(spawner, r.imu).await.unwrap();
    sensors::gps::init(spawner, r.gps);
    sensors::multimeter::init(spawner, i2c_device.clone())
        .await
        .unwrap();

    usb_interface::init(spawner, r.usb);

    if let Err(err) = black_box::init(spawner, r.sd_card).await {
        defmt::warn!("failed to initialize black box logging: {}", err);
    }

    let mut last = Instant::now();
    let mut idle_ticks: u64 = 0;

    let mut idle_led = gpio::Output::new(r.led.pin, gpio::Level::Low);
    loop {
        let before = Instant::now();
        cortex_m::asm::wfe();
        idle_ticks += (Instant::now() - before).as_micros();

        let now = Instant::now();
        if (now - last).as_micros() >= 1_000_000 {
            let total = (now - last).as_micros();
            let cpu = 100 - (idle_ticks * 100 / total);
            defmt::info!("cpu: {}%", cpu);
            idle_ticks = 0;
            last = now;

            let (gyro, accel) = critical_section::with(|cs| {
                (
                    sensors::imu::GYRO.borrow(cs).get(),
                    sensors::imu::ACCEL.borrow(cs).get(),
                )
            });
            defmt::debug!("gyro: {} {} {}", gyro.x, gyro.y, gyro.z);
            defmt::debug!("accel: {} {} {}", accel.x, accel.y, accel.z);

            idle_led.toggle();
        }
    }
}

type I2cBus = i2c::I2c<'static, I2C0, i2c::Async>;
type SharedI2cBus = Mutex<CriticalSectionRawMutex, I2cBus>;
pub type SharedI2cDevice = I2cDevice<'static, CriticalSectionRawMutex, I2cBus>;

fn init_i2c(p: I2cBusPeripherals) -> SharedI2cDevice {
    let mut config = i2c::Config::default();
    config.frequency = 400_000;
    config.sda_pullup = false;
    config.scl_pullup = false;
    let i2c_bus = i2c::I2c::new_async(p.i2c, p.scl_pin, p.sda_pin, Irqs, config);
    static I2C_BUS: StaticCell<SharedI2cBus> = StaticCell::new();
    I2cDevice::new(I2C_BUS.init(Mutex::new(i2c_bus)))
}

#[derive(Clone, Copy)]
pub struct SystemSpawner {
    pub realtime: SendSpawner,
    pub high_prio: SendSpawner,
    pub low_prio: SendSpawner,
}

static REALTIME_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static HIGH_PRIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static LOW_PRIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
fn SWI_IRQ_0() {
    unsafe { REALTIME_EXECUTOR.on_interrupt() };
}

#[cortex_m_rt::interrupt]
fn SWI_IRQ_1() {
    unsafe { HIGH_PRIO_EXECUTOR.on_interrupt() };
}

#[cortex_m_rt::interrupt]
fn SWI_IRQ_2() {
    unsafe { LOW_PRIO_EXECUTOR.on_interrupt() };
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
                 dma::InterruptHandler<DMA_CH5>;
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

// #[embassy_executor::task]
// async fn pid_loop(imu_p: Imu, motor_output: MotorOutput) {
//     let imu_spi = {
//         let mut config = spi::Config::default();
//         config.frequency = 2_000_000;
//         spi::Spi::new_blocking(
//             imu_p.spi,
//             imu_p.clk_pin,
//             imu_p.mosi_pin,
//             imu_p.miso_pin,
//             config,
//         )
//     };
//     let imu_spi_cs = gpio::Output::new(imu_p.cs_pin, gpio::Level::High);
//     let imu_spi_dev =
//         embedded_hal_bus::spi::ExclusiveDevice::new(imu_spi, imu_spi_cs, embassy_time::Delay)
//             .unwrap();
//
//     let mut esc_output = {
//         let mut config = pwm::Config::default();
//         config.top = ESC_PWM_TOP;
//         config.divider = FixedU16::from_num(1000 / ESC_PWM_PERIOD_MS);
//         let (Some(esc_output), None) =
//             pwm::Pwm::new_output_a(motor_output.slice_esc, motor_output.esc, config).split()
//         else {
//             unreachable!()
//         };
//         esc_output
//     };
//
//     let (mut left_elevon_output, mut right_elevon_output) = {
//         let mut config = pwm::Config::default();
//         config.top = ELEVON_PWM_TOP;
//         config.divider = FixedU16::from_num(1000 / ELEVON_PWM_PERIOD_MS);
//         let (Some(left_elevon), Some(right_elevon)) = pwm::Pwm::new_output_ab(
//             motor_output.slice12,
//             motor_output._1,
//             motor_output._2,
//             config,
//         )
//         .split() else {
//             unreachable!()
//         };
//
//         (left_elevon, right_elevon)
//     };
//
//     let mut imu = ICM42688::new(imu_spi_dev)
//         .initialize(
//             embassy_time::Delay,
//             icm426xx::Config {
//                 rate: icm426xx::OutputDataRate::Hz1000,
//                 ..Default::default()
//             },
//         )
//         .unwrap();
//
//     let mut madgwick_filter = ahrs::Madgwick::new(0.001, 0.01);
//
//     let mut last_tick = Instant::now();
//     loop {
//         let sample = loop {
//             match imu.read_sample() {
//                 Ok(Some((sample, more_to_read))) => {
//                     if more_to_read {
//                         imu.reset_fifo().ok();
//                     }
//                     break sample;
//                 }
//                 Ok(None) => (),
//                 Err(err) => {
//                     defmt::error!("failed to read imu: {}", err);
//                 }
//             }
//         };
//
//         let Some(gyro) = sample.gyro else {
//             defmt::error!("no sample from gyro");
//             continue;
//         };
//         let gyro = Vector3::new(gyro.0, gyro.1, gyro.2);
//
//         let Some(accel) = sample.accel else {
//             defmt::error!("no sample from accel");
//             continue;
//         };
//         let accel = Vector3::new(accel.0, accel.1, accel.2);
//
//         let Ok(orientation) = madgwick_filter.update_imu(&gyro, &accel) else {
//             defmt::error!("failed to update madgwick filter");
//             continue;
//         };
//
//         let throttle_value: f32;
//         let left_elevon_value: f32;
//         let right_elevon_value: f32;
//
//         let receiver_channels = RECEIVER_CHANNELS.lock(|channels| channels.get());
//         match RemoteCommand::from_channels(&receiver_channels) {
//             RemoteCommand::Manual {
//                 throttle,
//                 roll,
//                 pitch,
//                 ..
//             } => {
//                 throttle_value = throttle;
//                 left_elevon_value = (roll + pitch) / 2.;
//                 right_elevon_value = (roll - pitch) / 2.;
//             }
//             RemoteCommand::Assisted {
//                 speed_delta,
//                 roll,
//                 altitude_delta,
//             } => {
//                 throttle_value = 0.;
//                 left_elevon_value = 0.;
//                 right_elevon_value = 0.;
//             }
//             RemoteCommand::Autonomous => {
//                 throttle_value = 0.;
//                 left_elevon_value = 0.;
//                 right_elevon_value = 0.;
//             }
//         }
//
//         esc_output
//             .set_duty_cycle(us_to_duty(
//                 _0_1_to_us(throttle_value),
//                 ESC_PWM_PERIOD_MS,
//                 ESC_PWM_TOP,
//             ))
//             .ok();
//         left_elevon_output
//             .set_duty_cycle(us_to_duty(
//                 neg1_1_to_us(left_elevon_value),
//                 ELEVON_PWM_PERIOD_MS,
//                 ELEVON_PWM_TOP,
//             ))
//             .ok();
//         right_elevon_output
//             .set_duty_cycle(us_to_duty(
//                 neg1_1_to_us(right_elevon_value),
//                 ELEVON_PWM_PERIOD_MS,
//                 ELEVON_PWM_TOP,
//             ))
//             .ok();
//
//         let next_tick = last_tick + Duration::from_hz(1000);
//         Timer::at(next_tick).await;
//         last_tick = next_tick;
//     }
// }
