use super::vector::PhysVector2;

#[derive(Clone, Copy)]
pub struct PhysAABB {
    pub min: PhysVector2,
    pub max: PhysVector2,
}

impl PhysAABB {
    pub fn new(min: PhysVector2, max: PhysVector2) -> Self {
        Self { min: min, max: max }
    }
}