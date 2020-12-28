pub trait TwoAxisControls {
    /// Sample the latest input and represented as two normalized axes
    fn axes(&mut self) -> (f32, f32);
}


