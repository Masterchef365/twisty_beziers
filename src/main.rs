use anyhow::Result;
use klystron::{
    runtime_3d::{launch, App},
    DrawType, Engine, FramePacket, Object, Vertex, UNLIT_FRAG, UNLIT_VERT,
};
use nalgebra::{Matrix4, Point3, Vector3, Unit, UnitQuaternion};

struct MyApp {
    grid: Object,
    track: Object,
    time: f32,
}

impl App for MyApp {
    const NAME: &'static str = "MyApp";

    type Args = ();

    fn new(engine: &mut dyn Engine, _args: Self::Args) -> Result<Self> {
        let material = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Lines)?;

        // Grid
        let (vertices, indices) = grid(10, 1., [0.3; 3]);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let grid = Object {
            mesh,
            material,
            transform: Matrix4::identity(),
        };

        // Track
        let ctrlps = vec![
            TrackControl::new(Point3::new(0., 1., 0.), Vector3::new(1., 1., 1.)),
            TrackControl::new(Point3::new(1., 2., 3.), Vector3::new(2., -1., 1.)),
        ];

        let (vertices, indices) = track_trace(&ctrlps, 0.01, [0., 1., 0.]);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let track = Object {
            mesh,
            material,
            transform: Matrix4::identity(),
        };

        Ok(Self {
            track,
            grid,
            time: 0.0,
        })
    }

    fn next_frame(&mut self, engine: &mut dyn Engine) -> Result<FramePacket> {
        engine.update_time_value(self.time)?;
        self.time += 0.01;
        Ok(FramePacket {
            objects: vec![self.track, self.grid],
        })
    }
}

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

pub fn track_spline_deriv(begin: &TrackControl, end: &TrackControl, i: f32) -> Vector3<f32> {
    let iv = 1. - i; // i inverse
    let p0 = begin.position.coords;
    let p1 = begin.front_ctrlp().coords;
    let p2 = end.back_ctrlp().coords;
    let p3 = end.position.coords;
    (3. * iv.powf(2.) * (p1 - p0))
        + (6. * iv * i * (p2 - p1))
        + (3. * i.powf(2.) * (p3 - p2))
}

pub fn track_spline(begin: &TrackControl, end: &TrackControl, i: f32) -> Point3<f32> {
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

pub fn track_quat(begin: &TrackControl, end: &TrackControl, i: f32) -> UnitQuaternion<f32> {
    let deriv = track_spline_deriv(begin, end, i);
    let angle = lerp(begin.angle, end.angle, i);
    UnitQuaternion::from_axis_angle(&Unit::new_normalize(deriv), angle)
}

pub fn track_trace(segments: &[TrackControl], resolution: f32, color: [f32; 3]) -> (Vec<Vertex>, Vec<u16>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut push_point = |a: Point3<f32>| {
        vertices.push(Vertex { pos: *a.coords.as_ref(), color });
        indices.push(indices.len() as _);
    };

    for pair in segments.windows(2) {
        let (begin, end) = (&pair[0], &pair[1]);
        let mut i = 0.;
        while i < 1. {
            push_point(track_spline(begin, end, i));
            i += resolution;
            push_point(track_spline(begin, end, i));
        }
    }

    (vertices, indices)
}

fn main() -> Result<()> {
    let vr = std::env::args().skip(1).next().is_some();
    launch::<MyApp>(vr, ())
}

fn grid(size: i32, scale: f32, color: [f32; 3]) -> (Vec<Vertex>, Vec<u16>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut push_line = |a, b| {
        vertices.push(Vertex { pos: a, color });
        indices.push(indices.len() as _);
        vertices.push(Vertex { pos: b, color });
        indices.push(indices.len() as _);
    };

    let l = size as f32 * scale;
    for i in -size..=size {
        let f = i as f32 * scale;
        push_line([l, 0.0, f], [-l, 0.0, f]);
        push_line([f, 0.0, l], [f, 0.0, -l]);
    }

    (vertices, indices)
}

