use crate::manager::PhysBody;

use super::vector::PhysVector2;

pub struct PhysManifold {
    pub a_index: usize,
    pub b_index: usize,
    pub normal: PhysVector2,
    pub depth: f32,
    pub contact_one: PhysVector2,
    pub contact_two: PhysVector2,
    pub contact_count: u8,
}