use embassy_time::{Duration, Ticker};

pub async fn task() {
    let mut ticker = Ticker::every(Duration::from_hz(200));
    loop {
        ticker.next().await;
    }
}
