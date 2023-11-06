use super::vector::PhysVector2;

#[derive(Copy, Clone)]
pub struct PhysTransform {
    pub position: PhysVector2,
    pub comp_sin: f32,
    pub comp_cos: f32,
}

impl PhysTransform {
    pub fn new(position: PhysVector2, angle: f32) -> Self {
        Self { position: position, comp_sin: angle.sin(), comp_cos: angle.cos() }
    }
}

pub static ZERO_TRANSFORM: PhysTransform = PhysTransform { position: PhysVector2 { x: 0.0, y: 0.0 }, comp_sin: 0.0, comp_cos: 1.0 };