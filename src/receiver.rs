use core::cell::RefCell;

use cortex_m::prelude::_embedded_hal_serial_Read;
use crsf::PayloadDump;
use embassy_rp::{interrupt::{self, InterruptExt}, uart};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Ticker};

use crate::{Irqs, ReceiverPeripherals, sensors::multimeter, utils::SyncWatch};

pub static CHANNELS: SyncWatch<[u16; 16]> = SyncWatch::new([1500; 16]);
pub static LINK_STATS: SyncWatch<LinkStats> = SyncWatch::new(LinkStats {
    uplink_link_quality: 0,
    uplink_rssi: 0,
    downlink_link_quality: 0,
    downlink_rssi: 0,
});

#[derive(Clone, Copy)]
pub struct LinkStats {
    pub uplink_link_quality: u8,
    pub uplink_rssi: u8,
    pub downlink_link_quality: u8,
    pub downlink_rssi: u8,
}

pub fn init(spawner: crate::SystemSpawner, p: ReceiverPeripherals) {
    let mut config = uart::Config::default();
    config.baudrate = 115_200;

    let uart = uart::Uart::new_blocking(p.uart, p.tx_pin, p.rx_pin, config);
    let (uart_tx, uart_rx) = uart.split();

    embassy_rp::pac::UART0.uartimsc().write(|w| {
        w.set_rxim(true)
    });
    UART_RX.lock(|uart_rx_cell| uart_rx_cell.borrow_mut().replace(uart_rx));
    unsafe { interrupt::UART0_IRQ.enable() };

    let uart_tx = uart_tx.into_async(p.tx_dma, Irqs);

    spawner.default.spawn(telemetry_sender(uart_tx).unwrap());
}

static UART_RX: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<uart::UartRx<'static, uart::Blocking>>>,
> = Mutex::new(RefCell::new(None));

#[cortex_m_rt::interrupt]
fn UART0_IRQ() {
    static mut CRSF_PARSER: crsf::Parser = crsf::Parser::new(crsf::ParserConfig::default());

    let maybe_uart_rx = critical_section::with(|cs| UART_RX.borrow(cs).borrow_mut().take());
    let Some(mut uart_rx) = maybe_uart_rx else {
        defmt::warn!("uart_rx not available");
        return;
    };

    loop {
        let byte = match uart_rx.read() {
            Ok(byte) => byte,
            Err(err) => match err {
                embedded_hal_nb::nb::Error::WouldBlock => {
                    break;
                }
                embedded_hal_nb::nb::Error::Other(err) => {
                    CRSF_PARSER.reset();
                    defmt::warn!("failed to read from uart: {}", err);
                    continue;
                }
            },
        };

        let packet = match CRSF_PARSER.push_byte(byte) {
            Some(Ok(packet)) => packet,
            Some(Err(err)) => {
                defmt::warn!("failed to parse crsf packet: {}", err);
                continue;
            }
            None => continue,
        };

        match packet {
            crsf::Packet::RcChannelsPacked(raw_channels) => {
                let us_channels: [u16; 16] =
                    core::array::from_fn(|i| crsf_read_raw_rc(raw_channels.0[i]));
                critical_section::with(|cs| CHANNELS.set(cs, us_channels));
            }
            crsf::Packet::LinkStatistics(ls) => {
                let link_stats = LinkStats {
                    uplink_link_quality: ls.uplink_link_quality,
                    uplink_rssi: if ls.active_antenna == 1 {
                        ls.uplink_rssi_1
                    } else {
                        ls.uplink_rssi_2
                    },
                    downlink_link_quality: ls.downlink_link_quality,
                    downlink_rssi: ls.downlink_rssi,
                };
                critical_section::with(|cs| LINK_STATS.set(cs, link_stats));
            }
            _ => {}
        }
    }

    embassy_rp::pac::UART0.uarticr().write(|w| {
        w.set_rxic(true);
    });
    critical_section::with(|cs| UART_RX.borrow(cs).borrow_mut().replace(uart_rx));
}

const TELEMETRY_FREQ_HZ: u64 = 10;

#[embassy_executor::task]
async fn telemetry_sender(mut uart_tx: uart::UartTx<'static, uart::Async>) {
    let mut multimeter_watcher = multimeter::DATA.watcher();

    let mut ticker = Ticker::every(Duration::from_hz(TELEMETRY_FREQ_HZ));
    loop {
        ticker.next().await;

        let mut buf: [u8; crsf::MAX_PACKET_LEN] = [0; crsf::MAX_PACKET_LEN];

        if let Some((data, _)) = critical_section::with(|cs| multimeter_watcher.get_if_changed(cs))
        {
            let mut packet = crsf::BatterySensor::new();
            packet.set_voltage((data.voltage * 1e5) as u16); // convert from V to 10uV unit
            packet.set_current((data.current * 1e5) as u16); // convert from A to 10uA unit
            packet.set_capacity_used(data.capacity_used as u32); // already in mAh
            packet.set_remaining(0); // we don't have this info

            let len = packet.dump(&mut buf).unwrap(); // this can't fail as we have a big enough buffer
            if let Err(err) = uart_tx.write(&buf[..len]).await {
                defmt::warn!("failed to write to uart: {}", err);
            }
        }
    }
}

fn crsf_read_raw_rc(raw: u16) -> u16 {
    ((raw as u32) * 1024 / 1639 + 881) as u16
}
