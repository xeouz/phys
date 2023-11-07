use bevy::prelude::*;
use core::fmt;
use std::{f32::consts::PI, mem::discriminant, ops::{Add, Mul}};

use super::{vector::{PhysVector2, ZERO_VECTOR2}, world::{MAX_BODY_SIZE, MIN_BODY_SIZE, MAX_DENSITY, MIN_DENSITY}, clamp, transform::PhysTransform, vector_transform, aabb::PhysAABB};

#[derive(Debug)]
pub struct RigidBodyMoveError;

#[derive(Debug)]
pub struct RigidBodyAABBError;

#[derive(Debug)]
pub struct RigidBodyCalculateInertiaError;

pub enum RigidBodyType {  
    Rect { width: f32, height: f32, data: RectData },
    Circle { radius: f32 },
}

pub struct RigidBody {
    pub body_type: RigidBodyType,
    pub body_index: u8,
    pub transform_update_required: bool,
    pub aabb: PhysAABB,
    pub aabb_update_required: bool,

    force: PhysVector2,
    pub linear_velocity: PhysVector2,
    pub rotational_velocity: f32,
    pub rotation: f32,

    pub position: PhysVector2,
    pub density: f32,
    pub mass: f32,
    pub restitution: f32,
    pub area: f32,
    pub inverse_mass: f32,

    pub is_static: bool,
}

pub struct RectData { 
    pub vertices: Vec<PhysVector2>,
    pub transformed_vertices: Vec<PhysVector2>,
    pub triangles: Vec<f32>,
}

impl fmt::Display for RigidBodyType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            &Self::Circle { radius } => write!(f, "Circle[r:{}]", radius),
            &Self::Rect { width, height, data: _ } => write!(f, "Rect[w:{} h:{}]", width, height)
        }
    }
}

#[derive(Default)]
pub struct RigidBodyData {
    force: PhysVector2,
    pub linear_velocity: PhysVector2,
    pub rotational_velocity: f32,
    pub rotation: f32,

    pub position: PhysVector2,
    pub density: f32,
    pub mass: f32,
    pub restitution: f32,
    pub area: f32,
    pub inverse_mass: f32,

    pub is_static: bool,
}

impl RigidBody {
    pub fn new(body_type: RigidBodyType, aabb: PhysAABB, position: PhysVector2, density: f32, mass: f32, restitution: f32, area: f32, is_static: bool) -> Self {
        if density > MAX_DENSITY {
            panic!("Cannot create rigid body of density={} greater than MAX_DENSITY={}!", density, MAX_DENSITY);
        }
        else if density < MIN_DENSITY {
            panic!("Cannot create rigid body of density={} lesser than MIN_DENSITY={}!", density, MIN_DENSITY);
        }

        let inverse_mass = if !is_static {
            1.0 / mass
        } else {
            0.0
        };

        let body_index = get_body_index(&body_type);
        
        Self {
            position: position,
            body_type: body_type,
            body_index: body_index,
            transform_update_required: true,
            aabb: aabb,
            aabb_update_required: true,
            force: ZERO_VECTOR2,
            linear_velocity: ZERO_VECTOR2,
            rotation: 0.0,
            rotational_velocity: 0.0,
            density: density,
            mass: mass,
            restitution: clamp(restitution, 0.0, 1.0),
            area: area,
            inverse_mass: inverse_mass,
            is_static: is_static
        }
    }
    
    pub fn move_body(&mut self, amount: &PhysVector2) -> Result<(), RigidBodyMoveError> {
        self.transform_update_required = true;
        self.position = self.position.add(amount);

        Ok(())
    }

    pub fn move_to_body(&mut self, position: &PhysVector2) -> Result<(), RigidBodyMoveError> {
        self.transform_update_required = true;
        self.position = position.clone();

        Ok(())
    }

    pub fn rotate(&mut self, amount: f32) -> Result<(), RigidBodyMoveError> {
        self.transform_update_required = true;
        self.rotation += amount;

        Ok(())
    }

    pub fn get_transformed_vertices(&mut self) -> Option<&Vec<PhysVector2>> {
        match self.body_type {
            RigidBodyType::Rect { width, height, ref mut data } => {
                if self.transform_update_required {
                    let transform = PhysTransform::new(self.position, self.rotation);
                    for (i, vertex) in data.vertices.iter().enumerate() {
                        data.transformed_vertices[i] = vector_transform(*vertex, transform);
                    }
                }
                Some(&data.transformed_vertices)
            },

            _ => None
        }
    }

    pub fn get_body_index(&self) -> u8 {
        self.body_index
    }

    pub fn step(&mut self, delta_time: f32, iterations: usize, gravity: &PhysVector2) {
        if self.is_static {
            return;
        }

        let time = delta_time / iterations as f32;

        /*
        let acceleration = self.force.div(self.mass);
        self.linear_velocity = self.linear_velocity.add(&acceleration.mul(time));
        */

        self.linear_velocity = self.linear_velocity.add(&gravity.mul(time));
        self.position = self.position.add(&self.linear_velocity.mul(time));

        self.rotation = self.rotation.add(&self.rotational_velocity.mul(time));

        self.force = ZERO_VECTOR2;

        self.transform_update_required = true;
    }

    pub fn add_force(&mut self, force: PhysVector2) {
        self.force = force;
    }

    pub fn get_aabb(&mut self) -> Result<PhysAABB, RigidBodyAABBError> {
        if self.aabb_update_required {
            let (mut min_x, mut min_y) = (f32::MAX, f32::MAX);
            let (mut max_x, mut max_y) = (f32::MIN, f32::MIN);
    
            match &self.body_type {
                RigidBodyType::Circle { radius } => {
                    min_x = self.position.x - radius;
                    min_y = self.position.y - radius;
                    max_x = self.position.x + radius;
                    max_y = self.position.y + radius;
                    Ok(())
                },
                RigidBodyType::Rect { .. } => {
                    if let Some(vertices) = self.get_transformed_vertices() {
                        for vertex in vertices {
                            if vertex.x < min_x { min_x = vertex.x }
                            if vertex.y < min_y { min_y = vertex.y }
                            if vertex.x > max_x { max_x = vertex.x }
                            if vertex.y > max_y { max_y = vertex.y }
                        }
        
                        Ok(())
                    }
                    else {
                        Err(RigidBodyAABBError)
                    }
                },
                _ => Err(RigidBodyAABBError)
            }?;
    
            self.aabb = PhysAABB::new(PhysVector2 { x: min_x, y: min_y }, PhysVector2 { x: max_x, y: max_y });
        };
        
        self.aabb_update_required = true;
        Ok(self.aabb)
    }

    pub fn calculate_rotational_inertia(&mut self) -> Result<f32, RigidBodyCalculateInertiaError> {
        match &self.body_type {
            RigidBodyType::Rect { width, height, data } => {
                Ok((1.0 / 12.0) * self.mass * (width*width + height*height))
            },
    
            RigidBodyType::Circle { radius } => {
                Ok((1.0 / 2.0) * self.mass * radius * radius)

            }
    
            _ => {
                panic!()
            }
        }
    }
}

impl RigidBodyData {
    pub fn new(position: PhysVector2, density: f32, mass: f32, restitution: f32, area: f32, is_static: bool) -> Self {
        if density > MAX_DENSITY {
            panic!("Cannot create rigid body of density={} greater than MAX_DENSITY={}!", density, MAX_DENSITY);
        }
        else if density < MIN_DENSITY {
            panic!("Cannot create rigid body of density={} lesser than MIN_DENSITY={}!", density, MIN_DENSITY);
        }

        let inverse_mass = if !is_static {
            1.0 / mass
        } else {
            0.0
        };

        RigidBodyData {
            force: ZERO_VECTOR2,
            position: position, 
            linear_velocity: ZERO_VECTOR2,
            rotation: 0.0,
            rotational_velocity: 0.0,
            density: density,
            mass: mass,
            restitution: clamp(restitution, 0.0, 1.0),
            area: area,
            inverse_mass: inverse_mass,
            is_static: is_static
        }
    }

    pub fn move_body(&mut self, amount: &PhysVector2) -> Result<(), RigidBodyMoveError> {
        self.position = self.position.add(amount);

        Ok(())
    }

    pub fn move_to_body(&mut self, position: &PhysVector2) -> Result<(), RigidBodyMoveError> {
        self.position = position.clone();

        Ok(())
    }

    pub fn rotate(&mut self, amount: f32) -> Result<(), RigidBodyMoveError> {
        self.rotation += amount;

        Ok(())
    }

    pub fn step(&mut self, delta_time: f32, iterations: usize, gravity: &PhysVector2) {
        if self.is_static {
            return;
        }

        let time = delta_time / iterations as f32;

        /*
        let acceleration = self.force.div(self.mass);
        self.linear_velocity = self.linear_velocity.add(&acceleration.mul(time));
        */

        self.linear_velocity = self.linear_velocity.add(&gravity.mul(time));
        self.position = self.position.add(&self.linear_velocity.mul(time));

        self.rotation = self.rotation.add(&self.rotational_velocity.mul(time));

        self.force = ZERO_VECTOR2;
    }

    pub fn add_force(&mut self, force: PhysVector2) {
        self.force = force;
    }
}

impl RigidBodyType {
    pub const CIRCLE_INDEX: u8 = 0;
    pub const RECT_INDEX: u8 = 1;
}

fn create_box_vertices(width: f32, height: f32) -> [PhysVector2; 4] {
    let left = -width / 2.0;
    let right = left + width;
    let bottom = -height / 2.0;
    let top = bottom + height;

    [
        PhysVector2 { x: left, y: top },
        PhysVector2 { x: right, y: top },
        PhysVector2 { x: right, y: bottom },
        PhysVector2 { x: left, y: bottom },
    ]
}

fn create_box_triangles() -> [f32; 6] {
    [
        0.0, 1.0, 2.0, 0.0, 2.0, 3.0
    ]
}

fn get_body_index(body_type: &RigidBodyType) -> u8 {
    match body_type {
        &RigidBodyType::Circle { .. } => RigidBodyType::CIRCLE_INDEX,
        &RigidBodyType::Rect { .. } => RigidBodyType::RECT_INDEX,
    }
}

pub fn create_circle_body(position: PhysVector2, density: f32, restitution: f32, radius: f32, is_static: bool) -> RigidBody {
    let area = radius*radius*PI;

    if area > MAX_BODY_SIZE {
        panic!("Cannot create circle body of area={} greater than MAX_BODY_SIZE={}!", area, MAX_BODY_SIZE);
    }
    else if area < MIN_BODY_SIZE {
        panic!("Cannot create circle body of area={} smaller than MIN_BODY_SIZE={}!", area, MIN_BODY_SIZE);
    }

    let mass = area * density;

    let data = RigidBodyData::new(position, density, mass, restitution, area, is_static);
    let body_type = RigidBodyType::Circle { radius: radius };
    let aabb = PhysAABB::new(ZERO_VECTOR2, ZERO_VECTOR2);
    RigidBody::new(body_type, aabb, position, density, mass, restitution, area, is_static)
}

pub fn create_rect_body(position: PhysVector2, density: f32, restitution: f32, height: f32, width: f32, is_static: bool) -> RigidBody {
    let area = width*height;

    if area > MAX_BODY_SIZE {
        panic!("Cannot create rect body of area={} greater than MAX_BODY_SIZE={}!", area, MAX_BODY_SIZE);
    }
    else if area < MIN_BODY_SIZE {
        panic!("Cannot create rect body of area={} smaller than MIN_BODY_SIZE={}!", area, MIN_BODY_SIZE);
    }

    let mass = area * density;

    let vertices = create_box_vertices(width, height);
    let triangles = create_box_triangles();
    let transformed_vertices: [PhysVector2; 4] = Default::default();
    let rect_data = RectData { vertices: vertices.to_vec(), triangles: triangles.to_vec(), transformed_vertices: transformed_vertices.to_vec() };

    let data = RigidBodyData::new(position, density, mass, restitution, area, is_static);
    let body_type = RigidBodyType::Rect { width: width, height: height, data: rect_data };
    let aabb = PhysAABB::new(ZERO_VECTOR2, ZERO_VECTOR2);
    RigidBody::new(body_type, aabb, position, density, mass, restitution, area, is_static)
}

pub static ZERO_CIRCLE: RigidBodyType = RigidBodyType::Circle { radius: 0.0 };
pub static ZERO_RECT: RigidBodyType = RigidBodyType::Rect { width: 0.0, height: 0.0, data: RectData { vertices: vec![], transformed_vertices: vec![], triangles: vec![] } };