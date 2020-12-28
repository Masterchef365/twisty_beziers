use anyhow::Result;
use klystron::{
    runtime_2d::{event::WindowEvent, launch, App2D},
    DrawType, Engine, FramePacket, Matrix4, Object, Vertex, WinitBackend, UNLIT_FRAG, UNLIT_VERT,
};
use nalgebra::Vector4;

struct MyApp {
    bounds: Object,
    dot: Object,
}

const DOT_SCALE: f32 = 0.05;
const BASE_SCALE: f32 = 0.5;
fn scale_mat(scale: f32) -> Matrix4<f32> {
    Matrix4::from_diagonal(&Vector4::new(scale, scale, scale, 1.))
}

impl App2D for MyApp {
    const TITLE: &'static str = "Controls debugger";
    type Args = ();

    fn new(engine: &mut WinitBackend, _args: Self::Args) -> Result<Self> {
        // Bounding box
        let material = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Lines)?;

        let vertices = square_verts([1.; 3]);
        let indices = square_idx_lines();
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let bounds = Object {
            mesh,
            transform: Matrix4::identity(),
            material,
        };

        // Square
        let material = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Triangles)?;

        let vertices = square_verts([1., 0., 0.]);
        let indices = square_idx_tris();
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let dot = Object {
            mesh,
            transform: scale_mat(DOT_SCALE),
            material,
        };


        Ok(Self { bounds, dot })
    }

    fn event(&mut self, _event: &WindowEvent, _engine: &mut WinitBackend) -> Result<()> {
        Ok(())
    }

    fn frame(&self) -> FramePacket {
        FramePacket {
            base_transform: scale_mat(BASE_SCALE),
            objects: vec![self.bounds, self.dot],
        }
    }
}

fn square_verts(color: [f32; 3]) -> [Vertex; 4] {
    let vert = |x, y| Vertex::new([x, y, 0.], color);
    [vert(-1., -1.), vert(-1., 1.), vert(1., 1.), vert(1., -1.)]
}

fn square_idx_lines() -> [u16; 8] {
    [0, 1, 1, 2, 2, 3, 3, 0]
}

fn square_idx_tris() -> [u16; 6] {
    [0, 1, 2, 0, 2, 3]
}

fn main() -> Result<()> {
    launch::<MyApp>(())
}
