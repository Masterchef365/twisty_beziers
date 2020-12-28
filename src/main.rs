use anyhow::Result;
use klystron::{
    runtime_3d::{launch, App},
    DrawType, Engine, FramePacket, Object, Vertex, UNLIT_FRAG, UNLIT_VERT,
};
use nalgebra::{Matrix4, Point3, Vector3, Vector4};
use twisty_beziers::track::{self, TrackControl, TrackFollower, TrackSample};
use structopt::StructOpt;
use twisty_beziers::controls::{TwoAxisControls, GamepadAxes};
use wiiboard::WiiBoardRealtime;

struct MyApp {
    grid: Object,
    cart: Object,
    path: Object,
    ctrlps: Vec<TrackControl>,
    time: f32,
    controls: Box<dyn TwoAxisControls>,
    x_position: f32,
    opt: Opt,
}

const X_SENSITIVITY: f32 = 0.1;

impl App for MyApp {
    const NAME: &'static str = "MyApp";

    type Args = Opt;

    fn new(engine: &mut dyn Engine, opt: Self::Args) -> Result<Self> {
        let controls: Box<dyn TwoAxisControls> = match opt.wii {
            true => Box::new(WiiBoardRealtime::new(5, 5)),
            false => Box::new(GamepadAxes::new()?),
        };

        let lines = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Lines)?;

        // Grid
        let (vertices, indices) = grid(100, 1., [0.3; 3]);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let grid = Object {
            mesh,
            material: lines,
            transform: Matrix4::identity(),
        };

        // Track
        let ctrlps = vec![
            TrackControl::new(Point3::new(0., 0., 0.), Vector3::new(10., 0., 0.), 0.),
            TrackControl::new(Point3::new(20., 10., 0.), Vector3::new(10., 0., 0.), 0.),
            TrackControl::new(Point3::new(40., 20., 0.), Vector3::new(10., 0., 0.), std::f32::consts::PI),
            TrackControl::new(Point3::new(60., 00., 0.), Vector3::new(0., -10., 0.), std::f32::consts::PI),
            TrackControl::new(Point3::new(00., -20., 0.), Vector3::new(-10., 0., 0.), std::f32::consts::PI),
            TrackControl::new(Point3::new(0., 0., 0.), Vector3::new(10., 0., 0.), 0.),
        ];

        // Cart
        let triangles = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Triangles)?;
        let (vertices, indices) = rainbow_cube();
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let cart = Object {
            mesh,
            material: triangles,
            transform: Matrix4::identity(),
        };

        let floor = engine.add_material(
            UNLIT_VERT, 
            &std::fs::read("./shaders/floor.frag.spv")?, 
            DrawType::Triangles
        )?;

        // Path
        let (vertices, mut indices) = track_tess_path(&ctrlps, 4, 3.0, 0.5);
        double_side(&mut indices);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let path = Object {
            mesh,
            material: floor,
            transform: Matrix4::identity(),
        };

        Ok(Self {
            opt,
            x_position: 0.,
            controls,
            path,
            ctrlps,
            cart,
            grid,
            time: 0.0,
        })
    }

    fn next_frame(&mut self, engine: &mut dyn Engine) -> Result<FramePacket> {
        // Sample track position
        let sample = match track::sample_collection(&self.ctrlps, self.time) {
            Some(s) => s,
            None => {
                self.time = 0.;
                track::sample_collection(&self.ctrlps, self.time).unwrap()
            }
        };

        // Update X position based on game input
        let (x, y) = self.controls.axes().expect("Input device error");
        self.x_position += X_SENSITIVITY * x;

        // Update time (It's actually an index into the spline set but shhh)
        self.time += 0.1 / sample.derivative.magnitude();
        engine.update_time_value(self.time)?;

        // Determine transform for world
        let quat = sample.quaternion(&Vector3::x_axis());
        let cart_position = sample.position + road_norm(&sample) * self.x_position;
        let base_transform = 
            Matrix4::new_translation(&cart_position.coords) * quat.to_homogeneous();

        // Move the cart
        let size = 0.08;
        self.cart.transform =
            base_transform * Matrix4::from_diagonal(&Vector4::new(size, size, size, 1.));

        // Move the whole scene
        let base_transform = match self.opt.motion {
            false => Matrix4::identity(),
            true => base_transform.try_inverse().unwrap(),
        };

        Ok(FramePacket {
            base_transform,
            objects: vec![self.grid, self.cart, self.path],
        })
    }
}

pub fn road_norm(sample: &TrackSample) -> Vector3<f32> {
    let quat = sample.quaternion(&Vector3::x_axis());
    quat.transform_vector(&Vector3::z_axis())
}

pub fn double_side(indices: &mut Vec<u16>) {
    for i in (0..indices.len()).step_by(3) {
        indices.push(indices[i + 2]);
        indices.push(indices[i + 1]);
        indices.push(indices[i + 0]);
    }
}

pub fn track_tess_path(
    segments: &[TrackControl],
    lanes: i32,
    width: f32,
    resolution: f32,
) -> (Vec<Vertex>, Vec<u16>) {
    let mut vertices = Vec::new();
    let max_idx = segments.len() as f32;
    let mut follower = TrackFollower::new(segments, resolution);

    // Generate vertices
    let total_lanes = lanes * 2 + 1;
    let mut total_rows = 0;
    while let Some(sample) = follower.next() {
        let normal = road_norm(&sample) * width;
        let v = sample.index / max_idx;
        let w = follower.i;
        for lane in -lanes..=lanes {
            let u = lane as f32 / total_lanes as f32;
            let pos = sample.position + normal * (u * width);
            vertices.push(Vertex::new(*pos.coords.as_ref(), [(u + 0.5) / 2., v, w]));
        }
        total_rows += 1;
    }

    // Tesselate indices
    let mut indices = Vec::new();
    for row in 0..total_rows-1 {
        for col in 0..total_lanes-1 {
            let idx = (row * total_lanes + col) as u16;
            let total_lanes = total_lanes as u16;

            indices.push(idx);
            indices.push(idx + 1);
            indices.push(idx + total_lanes);

            indices.push(idx + total_lanes + 1);
            indices.push(idx + total_lanes);
            indices.push(idx + 1);
        }
    }

    (vertices, indices)
}

pub fn track_center_line(
    segments: &[TrackControl],
    resolution: f32,
    color: [f32; 3],
) -> (Vec<Vertex>, Vec<u16>) {
    let vertices: Vec<Vertex> = TrackFollower::new(segments, resolution)
        .map(|s| Vertex::new(*s.position.coords.as_ref(), color))
        .collect();
    let indices = (1..vertices.len() as u16 * 2).map(|i| i / 2).collect();
    (vertices, indices)
}

pub fn track_trace_away(
    segments: &[TrackControl],
    resolution: f32,
    away: f32,
    axis: Vector3<f32>,
    color: [f32; 3],
    vertices: &mut Vec<Vertex>,
) {
    for s in TrackFollower::new(segments, resolution) {
        vertices.push(Vertex::new(*s.position.coords.as_ref(), color));
        let quat = s.quaternion(&Vector3::y_axis());
        let v = s.position + quat.transform_vector(&axis) * away;
        vertices.push(Vertex::new(*v.coords.as_ref(), color));
    }
}

#[derive(Debug, StructOpt)]
#[structopt(name = "Twisty Beziers babey", about = "Beziers!")]
struct Opt {
    /// VR via OpenXR
    #[structopt(short, long)]
    vr: bool,

    /// Wii balance board mode
    #[structopt(short, long)]
    wii: bool,

    /// Attach yourself to the game and go weeeeee
    #[structopt(short, long)]
    motion: bool,
}

fn main() -> Result<()> {
    let opt = Opt::from_args();
    launch::<MyApp>(opt.vr, opt)
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

fn rainbow_cube() -> (Vec<Vertex>, Vec<u16>) {
    let vertices = vec![
        Vertex::new([-1.0, -1.0, -1.0], [0.0, 1.0, 1.0]),
        Vertex::new([1.0, -1.0, -1.0], [1.0, 0.0, 1.0]),
        Vertex::new([1.0, 1.0, -1.0], [1.0, 1.0, 0.0]),
        Vertex::new([-1.0, 1.0, -1.0], [0.0, 1.0, 1.0]),
        Vertex::new([-1.0, -1.0, 1.0], [1.0, 0.0, 1.0]),
        Vertex::new([1.0, -1.0, 1.0], [1.0, 1.0, 0.0]),
        Vertex::new([1.0, 1.0, 1.0], [0.0, 1.0, 1.0]),
        Vertex::new([-1.0, 1.0, 1.0], [1.0, 0.0, 1.0]),
    ];

    let indices = vec![
        3, 1, 0, 2, 1, 3, 2, 5, 1, 6, 5, 2, 6, 4, 5, 7, 4, 6, 7, 0, 4, 3, 0, 7, 7, 2, 3, 6, 2, 7,
        0, 5, 4, 1, 5, 0,
    ];

    (vertices, indices)
}
