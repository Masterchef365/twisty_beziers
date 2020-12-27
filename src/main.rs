use anyhow::Result;
use klystron::{
    runtime_3d::{launch, App},
    DrawType, Engine, FramePacket, Object, Vertex, UNLIT_FRAG, UNLIT_VERT,
};
use nalgebra::{Matrix4, Point3, Vector3, Vector4};
mod track;
use track::{TrackControl, TrackFollower};
use rand::distributions::{Uniform, Distribution};

struct MyApp {
    grid: Object,
    //track: Object,
    track_away: Object,
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

        let mut rng = rand::thread_rng();
        let angle = Uniform::new(0., std::f32::consts::TAU);
        let pos = Uniform::new(-1., 1.);
        let vect_rand = |r: &mut rand::rngs::ThreadRng| Vector3::new(pos.sample(r), pos.sample(r), pos.sample(r));

        // Track
        let mut ctrlps = Vec::new();
        let mut position = Point3::new(0., 0., 0.);
        for _ in 0..10 {
            let next_pos = position + vect_rand(&mut rng);
            ctrlps.push(TrackControl {
                position,
                direction: vect_rand(&mut rng),
                angle: angle.sample(&mut rng),
            });
            position = next_pos;
        }

        // Track trace
        /*
        let (vertices, indices) = track_center_line(&ctrlps, 0.01, [0., 1., 0.]);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let track = Object {
            mesh,
            material: lines,
            transform: Matrix4::identity(),
        };
        */

        // Away
        let mut vertices = Vec::new();
        let res = 0.01;
        let away = 0.5;
        track_trace_away(&ctrlps, res, away, *Vector3::x_axis(), [1., 0., 0.], &mut vertices);
        track_trace_away(&ctrlps, res, away, *Vector3::y_axis(), [0., 1., 0.], &mut vertices);
        track_trace_away(&ctrlps, res, away, *Vector3::z_axis(), [0., 0.5, 1.], &mut vertices);
        let indices: Vec<u16> = (0..vertices.len() as u16).collect();
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let scale = 4.;
        let track_away = Object {
            mesh,
            material: lines,
            transform: Matrix4::from_diagonal(&Vector4::new(scale, scale, scale, 1.)),
        };

        //let triangles = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Triangles)?;

        Ok(Self {
            ctrlps,
            track_away,
            //track,
            grid,
            time: 0.0,
        })
    }

    fn next_frame(&mut self, engine: &mut dyn Engine) -> Result<FramePacket> {
        engine.update_time_value(self.time)?;
        self.time += 0.005;
        Ok(FramePacket {
            objects: vec![self.track_away, self.grid],
        })
    }
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
