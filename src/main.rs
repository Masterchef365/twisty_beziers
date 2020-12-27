use anyhow::Result;
use klystron::{
    runtime_3d::{launch, App},
    DrawType, Engine, FramePacket, Object, Vertex, UNLIT_FRAG, UNLIT_VERT,
};
use nalgebra::{Matrix4, Point3, Vector3, Vector4};
mod track;
use track::{TrackControl, TrackFollower};

struct MyApp {
    grid: Object,
    track: Object,
    cart: Object,
    track_away: Object,
    path: Object,
    ctrlps: Vec<TrackControl>,
    time: f32,
}

impl App for MyApp {
    const NAME: &'static str = "MyApp";

    type Args = ();

    fn new(engine: &mut dyn Engine, _args: Self::Args) -> Result<Self> {
        let lines = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Lines)?;

        // Grid
        let (vertices, indices) = grid(10, 1., [0.3; 3]);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let grid = Object {
            mesh,
            material: lines,
            transform: Matrix4::identity(),
        };

        // Track
        let ctrlps = vec![
            TrackControl::new(Point3::new(0., 1., 0.), Vector3::new(1., 1., 1.), 0.),
            TrackControl::new(Point3::new(1., 4., 3.), Vector3::new(2., -1., 1.), -10.),
            TrackControl::new(Point3::new(-1., 2., 2.), Vector3::new(2., -1., 1.), -1.),
        ];

        // Track trace
        let (vertices, indices) = track_center_line(&ctrlps, 0.01, [0., 1., 0.]);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let track = Object {
            mesh,
            material: lines,
            transform: Matrix4::identity(),
        };

        // Away
        let mut vertices = Vec::new();
        track_trace_away(&ctrlps, 0.1, 0.1, *Vector3::x_axis(), [1., 0., 0.], &mut vertices);
        track_trace_away(&ctrlps, 0.1, 0.1, *Vector3::z_axis(), [0., 0.5, 1.], &mut vertices);
        let indices: Vec<u16> = (0..vertices.len() as u16).collect();
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let track_away = Object {
            mesh,
            material: lines,
            transform: Matrix4::identity(),
        };

        let triangles = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Triangles)?;

        // Cart
        let (vertices, indices) = rainbow_cube();
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let cart = Object {
            mesh,
            material: triangles,
            transform: Matrix4::identity(),
        };

        // Path
        let (vertices, indices) = track_tess_path(&ctrlps, 0.5, 0.01);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let path = Object {
            mesh,
            material: triangles,
            transform: Matrix4::identity(),
        };


        Ok(Self {
            path,
            ctrlps,
            cart,
            track_away,
            track,
            grid,
            time: 0.0,
        })
    }

    fn next_frame(&mut self, engine: &mut dyn Engine) -> Result<FramePacket> {
        engine.update_time_value(self.time)?;
        self.time += 0.005;

        let sample = match track::sample_collection(&self.ctrlps, self.time) {
            Some(s) => s,
            None => {
                self.time = 0.;
                track::sample_collection(&self.ctrlps, self.time).unwrap()
            }
        };
        let quat = sample.quaternion(&Vector3::y_axis());
        let size = 0.08;
        self.cart.transform = Matrix4::new_translation(&sample.position.coords)
            * quat.to_homogeneous()
            * Matrix4::from_diagonal(&Vector4::new(size, size, size, 1.));

        Ok(FramePacket {
            //objects: vec![self.track, self.track_away, self.grid, self.cart, self.path],
            objects: vec![self.grid, self.cart, self.path],
        })
    }
}


pub fn track_tess_path(
    segments: &[TrackControl],
    width: f32,
    resolution: f32,
) -> (Vec<Vertex>, Vec<u16>) {
    let mut vertices = Vec::new();
    let max_idx = segments.len() as f32;
    for sample in TrackFollower::new(segments, resolution) {
        let quat = sample.quaternion(&Vector3::y_axis());
        let normal = quat.transform_vector(&Vector3::x_axis()) * width;
        let left = (sample.position - normal).coords;
        let v = sample.index / max_idx;
        vertices.push(Vertex::new(*left.as_ref(), [0., v, 0.]));
        let right = (sample.position + normal).coords;
        vertices.push(Vertex::new(*right.as_ref(), [1., v, 0.]));
    }

    let mut indices = Vec::new();
    for i in (2..vertices.len() as u16).step_by(2) {
        indices.extend_from_slice(&[i, i - 1, i - 2]);
        indices.extend_from_slice(&[i - 1, i, i + 1]);
    }

    (vertices, indices)
}


pub fn track_center_line(
    segments: &[TrackControl],
    resolution: f32,
    color: [f32; 3],
) -> (Vec<Vertex>, Vec<u16>) {
    let vertices: Vec<Vertex> = 
        TrackFollower::new(segments, resolution)
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
