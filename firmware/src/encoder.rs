pub trait RotaryEncoder {
    type Error;

    /// Returns rotation in radians
    #[allow(async_fn_in_trait)]
    async fn rotation(&mut self) -> Result<f32, Self::Error>;
}

pub struct MagneticEncoder<I> {
    pub channel: I,
}

impl<I> RotaryEncoder for MagneticEncoder<I>
where
    I: embedded_hal_async::i2c::I2c,
{
    type Error = I::Error;
    async fn rotation(&mut self) -> Result<f32, Self::Error> {
        let msg = [0x0Cu8];
        let mut response = [0_u8; 2];
        self.channel.write_read(0x36u8, &msg, &mut response).await?;
        let rot = u16::from_be_bytes(response);
        let mut angle = rot as f32 * (core::f32::consts::PI * 2.0 / 4096.0);
        if angle >= core::f32::consts::PI {
            angle -= 2.0 * core::f32::consts::PI;
        }
        Ok(angle)
    }
}
