use smart_leds::RGB8;

pub trait LedStrip {
    async fn write<const N: usize>(&mut self, colors: &[RGB8; N]);
}

#[cfg(feature = "rp")]
impl<
    'a,
    P: embassy_rp::pio::Instance,
    const S: usize,
    ORDER: embassy_rp::pio_programs::ws2812::RgbColorOrder,
> LedStrip for embassy_rp::pio_programs::ws2812::PioWs2812<'a, P, S, ORDER>
{
    async fn write<const N: usize>(&mut self, colors: &[RGB8; N]) {
        self.write_slice(colors).await;
    }
}
