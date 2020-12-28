use anyhow::Result;
use klystron::{
    runtime_2d::{event::WindowEvent, launch, App2D},
    DrawType, Engine, FramePacket, Matrix4, Object, Vertex, WinitBackend, UNLIT_FRAG, UNLIT_VERT,
};
use nalgebra::{Vector4, Vector3};
use twisty_beziers::controls::{TwoAxisControls, GamepadAxes};
use wiiboard::WiiBoardRealtime;

struct MyApp {
    bounds: Object,
    dot: Object,
    cross: Object,
    controls: Box<dyn TwoAxisControls>,
}

const CROSS_SCALE: f32 = 0.1;
const DOT_SCALE: f32 = 0.03;
const BASE_SCALE: f32 = 0.5;
fn scale_mat(scale: f32) -> Matrix4<f32> {
    Matrix4::from_diagonal(&Vector4::new(scale, scale, scale, 1.))
}

impl App2D for MyApp {
    const TITLE: &'static str = "Controls debugger";
    type Args = bool;

    fn new(engine: &mut WinitBackend, wii: Self::Args) -> Result<Self> {
        let controls: Box<dyn TwoAxisControls> = match wii {
            true => Box::new(WiiBoardRealtime::new(5, 5)),
            false => Box::new(GamepadAxes::new()?),
        };

        // Bounding box
        let lines = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Lines)?;

        let vertices = square_verts([1.; 3]);
        let indices = square_idx_lines();
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let bounds = Object {
            mesh,
            transform: Matrix4::identity(),
            material: lines,
        };

        // Cross
        let (vertices, indices) = cross([1.; 3]);
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let cross = Object {
            mesh,
            transform: scale_mat(CROSS_SCALE),
            material: lines,
        };

        // Square
        let tris = engine.add_material(UNLIT_VERT, UNLIT_FRAG, DrawType::Triangles)?;

        let vertices = square_verts([1., 0., 0.]);
        let indices = square_idx_tris();
        let mesh = engine.add_mesh(&vertices, &indices)?;

        let dot = Object {
            mesh,
            transform: scale_mat(DOT_SCALE),
            material: tris,
        };

        Ok(Self { bounds, dot, cross, controls })
    }

    fn event(&mut self, _event: &WindowEvent, _engine: &mut WinitBackend) -> Result<()> {
        Ok(())
    }

    fn frame(&mut self) -> FramePacket {
        let (x, y) = self.controls.axes().expect("Input device error");
        self.dot.transform = Matrix4::new_translation(&Vector3::new(x, -y, 0.)) * scale_mat(DOT_SCALE);
        FramePacket {
            base_transform: scale_mat(BASE_SCALE),
            objects: vec![self.bounds, self.dot, self.cross],
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

fn cross(color: [f32; 3]) -> ([Vertex; 4], [u16; 4]) {
    let vert = |x, y| Vertex::new([x, y, 0.], color);
    (
        [vert(0., -1.), vert(0., 1.), vert(1., 0.), vert(-1., 0.)],
        [0, 1, 2, 3]
    )
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let wii = args.next().is_some();
    launch::<MyApp>(wii)
}
