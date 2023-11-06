use core::fmt;

#[derive(Copy, Clone)]
pub struct PhysVector2 {
    pub x: f32,
    pub y: f32,
}

impl Default for PhysVector2 {
    fn default() -> Self {
        PhysVector2 { x: 0.0, y: 0.0 }
    }
}

impl PhysVector2 {
    pub fn neg(&self) -> Self {
        Self { x: self.x*-1.0, y: self.y*-1.0 }
    }

    pub fn eq(&self, other: &Self) -> bool {
        (self.x == other.x) && (self.y == other.y)
    }

    pub fn add(&self, other: &Self) -> Self {
        Self { x: self.x + other.x, y: self.y + other.y }
    }

    pub fn sub(&self, other: &Self) -> Self {
        Self { x: self.x - other.x, y: self.y - other.y }
    }

    pub fn mul(&self, other: f32) -> Self {
        Self { x: self.x * other, y: self.y * other }
    }

    pub fn div(&self, other: f32) -> Self {
        Self { x: self.x / other, y: self.y / other }
    }
}

impl fmt::Display for PhysVector2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vector2[x:{} y:{}]", self.x, self.y)
    }
}

pub static ZERO_VECTOR2: PhysVector2 = PhysVector2 { x:0.0, y:0.0 };