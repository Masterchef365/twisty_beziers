use anyhow::{Result, format_err, Context};
use gilrs::{Gilrs, GamepadId, Axis};
use wiiboard::WiiBoardRealtime;

pub trait TwoAxisControls {
    /// Sample the latest input and represented as two normalized axes
    fn axes(&mut self) -> Result<(f32, f32)>;
}

pub struct Dummy;
impl TwoAxisControls for Dummy {
    fn axes(&mut self) -> Result<(f32, f32)> {
        Ok((0., 0.))
    }
}

pub struct GamepadAxes {
    gilrs: Gilrs,
    gamepad: GamepadId,
}

impl GamepadAxes {
    pub fn new() -> Result<Self> {
        let gilrs = Gilrs::new().map_err(|e| format_err!("gilrs failed to init {}", e))?;
        let (gamepad, _) = gilrs.gamepads().next().context("No gamepads found")?;
        Ok(Self { gilrs, gamepad })
    }
}

impl TwoAxisControls for GamepadAxes {
    fn axes(&mut self) -> Result<(f32, f32)> {
        self.gilrs.next_event();
        self.gilrs.next_event();
        let x = self
            .gilrs
            .gamepad(self.gamepad)
            .axis_data(Axis::LeftStickX)
            .map(|v| v.value())
            .unwrap_or(0.0);
        let y = self
            .gilrs
            .gamepad(self.gamepad)
            .axis_data(Axis::LeftStickY)
            .map(|v| v.value())
            .unwrap_or(0.0);
        Ok((x, y))
    }
}

impl TwoAxisControls for WiiBoardRealtime {
    fn axes(&mut self) -> Result<(f32, f32)> {
         if let Some(data) = self.poll()? {
            let total = data.top_left + data.top_right + data.bottom_left + data.bottom_right;
            if total > 0.0 {
                let x = ((data.top_right + data.bottom_right) / total) * 2. - 1.;
                let y = ((data.top_left + data.top_right) / total) * 2. - 1.;
                //return Ok((x, y));
                return Ok((y, x));
            }
        }

        Ok((0., 0.))
    }
}

