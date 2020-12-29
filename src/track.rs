use nalgebra::{Point3, Vector3, UnitQuaternion, Unit};

/// Control point for a track
pub struct TrackControl {
    pub position: Point3<f32>,
    pub direction: Vector3<f32>,
    pub angle: f32,
}

impl TrackControl {
    /// Create a new TrackControl
    pub fn new(position: Point3<f32>, direction: Vector3<f32>, angle: f32) -> Self {
        Self {
            position,
            direction,
            angle,
        }
    }

    /// The control point in front of this track control
    pub fn front_ctrlp(&self) -> Point3<f32> {
        self.position + self.direction
    }

    /// The control point behind this track control
    pub fn back_ctrlp(&self) -> Point3<f32> {
        self.position - self.direction
    }
}

/// Sample of a track
pub struct TrackSample {
    pub position: Point3<f32>,
    pub derivative: Vector3<f32>,
    pub angle: f32,
    pub index: f32,
}

impl TrackSample {
    pub fn quaternion(&self, axis: &Vector3<f32>) -> UnitQuaternion<f32> {
        UnitQuaternion::from_axis_angle(&Unit::new_normalize(self.derivative), self.angle) *
        UnitQuaternion::rotation_between(axis, &self.derivative).unwrap()
    }
}

// Smoothe step from [0..1] to [0..1]
fn smooth_step(i: f32) -> f32 {
    i * i * (3. - 2. * i)
}

/// Sample between two track controls
pub fn sample(begin: &TrackControl, end: &TrackControl, i: f32) -> TrackSample {
    let position = spline(begin, end, i);
    let derivative = spline_deriv(begin, end, i);
    let angle = lerp(begin.angle, end.angle, smooth_step(i));
    TrackSample {
        position,
        derivative,
        angle,
        index: i,
    }
}

/// Sample from a collection of track controls
pub fn sample_collection(controls: &[TrackControl], i: f32) -> Option<TrackSample> {
    use std::convert::TryInto;
    let base: usize = (i as i64).try_into().ok()?;
    let begin = controls.get(base)?;
    let end = controls.get(base+1)?;
    let mut sample = sample(begin, end, i.fract());
    sample.index = i;
    Some(sample)
}

/// Derivative between two track controls
pub fn spline_deriv(begin: &TrackControl, end: &TrackControl, i: f32) -> Vector3<f32> {
    let iv = 1. - i; // i inverse
    let p0 = begin.position.coords;
    let p1 = begin.front_ctrlp().coords;
    let p2 = end.back_ctrlp().coords;
    let p3 = end.position.coords;
    (3. * iv.powf(2.) * (p1 - p0))
        + (6. * iv * i * (p2 - p1))
        + (3. * i.powf(2.) * (p3 - p2))
}

/// Position between two track controls
pub fn spline(
    begin: &TrackControl,
    end: &TrackControl,
    i: f32,
) -> Point3<f32> {
    let iv = 1. - i; // i inverse
    let p0 = begin.position.coords;
    let p1 = begin.front_ctrlp().coords;
    let p2 = end.back_ctrlp().coords;
    let p3 = end.position.coords;
    let coords = (iv.powf(3.) * p0)
        + (3. * iv.powf(2.) * i * p1)
        + (3. * iv * i.powf(2.) * p2)
        + (i.powf(3.) * p3);
    Point3 { coords }
}

fn lerp(a: f32, b: f32, i: f32) -> f32 {
    a * (1. - i) + b * i
}

/// Track follower
pub struct TrackFollower<'a> {
    controls: &'a [TrackControl],
    pub i: f32,
    pub rate: f32,
}

impl<'a> TrackFollower<'a> {
    /// Create a new follower at `i = 0`
    pub fn new(controls: &'a [TrackControl], rate: f32) -> Self {
        Self { controls, rate, i: 0. }
    }
}

impl Iterator for TrackFollower<'_> {
    type Item = TrackSample;

    fn next(&mut self) -> Option<Self::Item> {
        let sample = sample_collection(self.controls, self.i)?;
        self.i += self.rate / sample.derivative.magnitude();
        Some(sample)
    }
}

