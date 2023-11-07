use std::collections::HashMap;

use bevy::prelude::*;

use crate::{math::{rigidbody::{RigidBody, RigidBodyType, RigidBodyMoveError, RigidBodyAABBError}, vector::{PhysVector2, ZERO_VECTOR2}, intersect_circle, intersect_polygons, intersect_circle_polygon, vector_dot, intersect_polygons_centered, intersect_circle_polygon_centered, clamp, manifold::PhysManifold, find_contact_points, aabb::PhysAABB, intersect_aabb}, spawner::{spawn_rect, spawn_rect_default, SpawnError, spawn_circle_default, SpawnMetadata, SpawnArgs, spawn_circle, DEFAULT_STROKE_COLOR, DEFAULT_FILL_COLOR, DEFAULT_STROKE_WIDTH}};

#[derive(Resource)]
pub struct PhysBody {
    pub body: RigidBody,
    pub entity: Entity,
}

impl PhysBody {
    pub fn new(body: RigidBody, entity: Entity) -> Self {
        Self { body: body, entity: entity }
    }

    pub fn get_body_index(&self) -> u8 {
        self.body.get_body_index()
    }

    pub fn get_aabb(&mut self) -> Result<PhysAABB, RigidBodyAABBError> {
        self.body.get_aabb()
    }

    pub fn step(&mut self, time: f32, iterations: usize, gravity: &PhysVector2) {
        self.body.step(time, iterations, gravity);
    }

    pub fn move_body(&mut self, new_position: &PhysVector2) -> Result<(), RigidBodyMoveError>{
        self.body.move_body(&new_position)
    }

    pub fn move_to_body(&mut self, new_position: &PhysVector2) -> Result<(), RigidBodyMoveError>{
        self.body.move_to_body(&new_position)
    }

    pub fn rotate(&mut self, amount: f32) -> Result<(), RigidBodyMoveError> {
        self.body.rotate(amount)
    }

    pub fn add_force(&mut self, force: PhysVector2) {
        self.body.add_force(force)
    }
}

#[derive(Default, Resource)]
pub struct WorldData {
    pub selected_body: Option<(Entity, usize)>,
}

impl WorldData {
    pub fn select(&mut self, body: Entity, index: usize) -> &mut (Entity, usize) {
        self.selected_body.insert((body, index))
    }
}

pub struct Extents {
    pub left: f32,
    pub right: f32,
    pub bottom: f32,
    pub top: f32,
}

#[derive(Resource)]
pub struct PhysManager {
    gravity: PhysVector2,
    pub world_extents: Extents, // LEFT RIGHT BOTTOM TOP
    pub bodies: Vec<(Entity, PhysBody)>,
    pub contact_list_render_pool: Vec<Entity>,
}

impl Default for PhysManager {
    fn default() -> Self {
        Self::new(PhysVector2 { x: 0.0, y: -98.1*5.0 }, Extents { left: -600.0, right: 600.0, bottom: -350.0, top: 350.0 })
    }
}

impl PhysManager {
    pub const MIN_ITERATIONS: usize = 1;
    pub const MAX_ITERATIONS: usize = 128;
    
    pub fn new(gravity: PhysVector2, world_extents: Extents) -> Self {
        Self { gravity: gravity, world_extents: world_extents, bodies: vec![], contact_list_render_pool: vec![] }
    }
    
    pub fn add_bodies(&mut self, commands: &mut Commands, bodies: Vec<(Option<SpawnArgs>, RigidBody)>) -> Result<(), SpawnError> {
        for (spawn_args, body) in bodies {
            let args = if let Some(a) = spawn_args { a } else {
                let mut s = SpawnArgs::default();
                if body.is_static {
                    s.stroke_color = Color::rgb_u8(0, 0, 0);
                    s.stroke_width = 4.0;
                }
                s
            };
            self.add_body(commands, body, args)?;
        };

        Ok(())
    }

    pub fn add_body(&mut self, commands: &mut Commands, body: RigidBody, spawn_args: SpawnArgs) -> Result<(), SpawnError>  {
        let (entity, n_render) = match &body.body_type {
            RigidBodyType::Rect { width, height, data: _ } => {
                (spawn_rect(commands, &body.position, *height, *width, spawn_args)?, 2)
            },
            RigidBodyType::Circle { radius } => {
                (spawn_circle(commands, &body.position, *radius, spawn_args)?, 1)
            }
        };

        for _i in 0..n_render {
            let contact_point_entity = spawn_rect(commands, &ZERO_VECTOR2, 10.0, 10.0, SpawnArgs {
                z_value: 2.0,
                fill_color: *DEFAULT_FILL_COLOR,
                stroke_color: *DEFAULT_STROKE_COLOR,
                stroke_width: DEFAULT_STROKE_WIDTH,
                is_visible: false,
            }).unwrap();

            self.contact_list_render_pool.push(contact_point_entity);
        }
        self.bodies.push((entity, PhysBody::new(body, entity)));

        Ok(())
    }

    pub fn remove_body(&mut self, commands: &mut Commands, index: usize) {
        let (entity, _body) = self.bodies.swap_remove(index);
        let contact_render_entity = self.contact_list_render_pool.pop().unwrap();
        commands.entity(entity).despawn();
        commands.entity(contact_render_entity).despawn();
    }

    pub fn get_body(&self, index: usize) -> Option<&(Entity, PhysBody)> {
        self.bodies.get(index)
    }

    pub fn get_body_mut(&mut self, index: usize) -> Option<&mut (Entity, PhysBody)> {
        self.bodies.get_mut(index)
    }

    pub fn get_two_body_mut(&mut self, a_index: usize, b_index: usize) -> Option<(&mut (Entity, PhysBody), &mut (Entity, PhysBody))> {
        if a_index < b_index {
            // `i` is in the left half
            let (left, right) = self.bodies.split_at_mut(b_index);
            Some((&mut left[a_index], &mut right[0]))
        } else if a_index == b_index {
            // cannot obtain two mutable references to the
            // same element
            None
        } else {
            // `i` is in the right half
            let (left, right) = self.bodies.split_at_mut(a_index);
            Some((&mut right[0], &mut left[b_index]))
        }
    }

    fn collide_bodies(&mut self, a_index: usize, b_index: usize) -> ((&mut PhysBody, &mut PhysBody), (bool, PhysVector2, f32)) {
        let ((_, first_body), (_, second_body)) = self.get_two_body_mut(a_index, b_index).unwrap();
        
        let mut success: bool = false;
        let mut normal: PhysVector2 = ZERO_VECTOR2;
        let mut depth: f32 = 0.0;

        // CIRCLE AND CIRCLE
        if first_body.get_body_index() == second_body.get_body_index()
        && first_body.get_body_index() == RigidBodyType::CIRCLE_INDEX {
            let radius_a = match first_body.body.body_type { RigidBodyType::Circle { radius } => radius, _ => 0.0 };
            let radius_b = match second_body.body.body_type { RigidBodyType::Circle { radius } => radius, _ => 0.0 };
            (success, normal, depth) = intersect_circle(
                &first_body.body.position, 
                radius_a,
                &second_body.body.position,
                radius_b);
        }
        
        // RECT AND RECT
        else if first_body.get_body_index() == second_body.get_body_index()
        && first_body.get_body_index() == RigidBodyType::RECT_INDEX {
            // let (width_a, height_a, data_a) = match &first_body.body.body_type { RigidBodyType::Rect { width, height, data } => { Some((width,height,data)) }, _ => None }.unwrap();
            // let (width_b, height_b, data_b) = match &second_body.body.body_type { RigidBodyType::Rect { width, height, data } => { Some((width,height,data)) }, _ => None }.unwrap();
            
            let pa = first_body.body.position;
            let pb = second_body.body.position;
            let va= first_body.body.get_transformed_vertices().unwrap();
            let vb = second_body.body.get_transformed_vertices().unwrap();
            (success, normal, depth) = intersect_polygons_centered(va, vb, pa, pb);
        }
        
        // RECT AND CIRCLE
        else if first_body.get_body_index() == RigidBodyType::RECT_INDEX && second_body.get_body_index() == RigidBodyType::CIRCLE_INDEX {
            let radius = match second_body.body.body_type { RigidBodyType::Circle { radius } => radius, _ => 0.0 };
            let center = &second_body.body.position;
            let polygon_center = first_body.body.position;
            let vertices = first_body.body.get_transformed_vertices().unwrap();

            (success, normal, depth) = intersect_circle_polygon_centered(center, radius, vertices, polygon_center);
            normal = normal.neg();
        }

        // CIRCLE AND RECT
        else if first_body.get_body_index() == RigidBodyType::CIRCLE_INDEX && second_body.get_body_index() == RigidBodyType::RECT_INDEX {
            let radius = match first_body.body.body_type { RigidBodyType::Circle { radius } => radius, _ => 0.0 };
            let center = &first_body.body.position;
            let polygon_center = second_body.body.position;
            let vertices = second_body.body.get_transformed_vertices().unwrap();

            (success, normal, depth) = intersect_circle_polygon_centered(center, radius, vertices, polygon_center);
        }

        // UNKNOWN           
        else {
            println!("Cannot compute collision between {} and {}", first_body.body.body_type, second_body.body.body_type);
        }

        ((first_body, second_body), (success, normal, depth))
    }

    fn broad_phase(&mut self) -> Result<Vec<(usize, usize)>, RigidBodyMoveError> {
        let length = self.bodies.len();
        let mut contact_pairs: Vec<(usize, usize)> = vec![];
        for i in 0..length-1 {
            for j in i+1..length {
                let ((_, first_body), (_, second_body)) = self.get_two_body_mut(i, j).unwrap();
                if first_body.body.is_static && second_body.body.is_static {
                    continue;
                }

                let (first_aabb, second_aabb) = (first_body.get_aabb().unwrap(), second_body.get_aabb().unwrap());
                if !intersect_aabb(&first_aabb, &second_aabb) {
                    continue;
                }

                contact_pairs.push((i, j));
            }
        }
    
        Ok(contact_pairs)
    }

    fn narrow_phase(&mut self, contact_pairs: Vec<(usize, usize)>, entity_q: &mut Query<(&mut Transform, &mut Visibility, &mut SpawnMetadata)>) -> Result<(), RigidBodyMoveError> {
        let mut contact_points: Vec<PhysManifold> = vec![];
        
        for contact_pair in contact_pairs.iter() {
            let ((first_body, second_body), (success, normal, depth)) = self.collide_bodies(contact_pair.0, contact_pair.1);
                
            if success {
                if first_body.body.is_static {
                    second_body.move_body(&normal.mul(depth))?;
                }
                else if second_body.body.is_static {
                    first_body.move_body(&normal.neg().mul(depth))?;
                }
                else {
                    first_body.move_body(&normal.neg().mul(depth / 2.0))?;
                    second_body.move_body(&normal.mul(depth / 2.0))?;
                }
                
                let (contact_one, contact_two, contact_count) = find_contact_points(&mut first_body.body, &mut second_body.body);
                let contact = PhysManifold {
                    a_index: contact_pair.0, b_index: contact_pair.1, normal: normal, depth: depth, contact_one: contact_one, contact_two: contact_two, contact_count: contact_count
                };
                self.resolve_collision(&contact);
                contact_points.push(contact)
            }

        }

        let mut next_contact: Option<PhysVector2> = None;
        let mut indx = 0; // keep a manual index
        for render_entity in self.contact_list_render_pool.iter() {
            if next_contact.is_some() {
                if let Ok((mut transform, mut render_vis, _)) = entity_q.get_mut(*render_entity) {
                    let point = next_contact.unwrap();
                    (transform.translation.x, transform.translation.y) = (point.x, point.y);
                    *render_vis = Visibility::Visible;
                }
                next_contact = None;
                continue;
            }

            if let Ok((mut transform, mut render_vis, _)) = entity_q.get_mut(*render_entity) {
                *render_vis = Visibility::Hidden;
                if let Some(contact) = contact_points.get(indx) {
                    if contact.contact_count > 0 {
                        let point = contact.contact_one;
                        (transform.translation.x, transform.translation.y) = (point.x, point.y);
                        *render_vis = Visibility::Visible;

                        if contact.contact_count == 2 {
                            next_contact = Some(contact.contact_two);
                        }
                    }
                }
            }

            indx += 1;
        }

        Ok(())
    }

    pub fn resolve_collision(&mut self, contact: &PhysManifold) {
        let ((_, a), (_, b)) = self.get_two_body_mut(contact.a_index, contact.b_index).unwrap();
        
        let relative_velocity = b.body.linear_velocity.sub(&a.body.linear_velocity);

        if vector_dot(&relative_velocity, &contact.normal) > 0.0 { // Moving apart
            return;
        }

        let e = f32::min(a.body.restitution, b.body.restitution);

        let numerator = -(1.0 + e) * vector_dot(&relative_velocity, &contact.normal);
        let denominator = a.body.inverse_mass + b.body.inverse_mass;
        let j = numerator / denominator;

        let impulse = contact.normal.mul(j);
        a.body.linear_velocity = a.body.linear_velocity.sub(&impulse.mul(a.body.inverse_mass));
        b.body.linear_velocity = b.body.linear_velocity.add(&impulse.mul(b.body.inverse_mass));
    }

    pub fn step(&mut self, commands: &mut Commands, mut entity_q: Query<(&mut Transform, &mut Visibility, &mut SpawnMetadata)>, time: f32, iterations: usize) {
        let num_iterations = clamp(iterations, Self::MIN_ITERATIONS, Self::MAX_ITERATIONS);

        for i in 0..num_iterations {
            self.step_bodies(commands, &mut entity_q, time, iterations);

            //--- COLLISION DETECTION ---//
            let res = self.broad_phase().unwrap();
            self.narrow_phase(res, &mut entity_q);
            //--- COLLISION DETECTION ---//
        }
    }

    fn step_bodies(&mut self, commands: &mut Commands, entity_q: &mut Query<(&mut Transform, &mut Visibility, &mut SpawnMetadata)>, time: f32, iterations: usize) {
        let mut remove_indices: Vec<usize> = vec![];
        for (index, (_, body)) in self.bodies.iter_mut().enumerate() {
            body.step(time, iterations, &self.gravity);

            let aabb = body.body.get_aabb().unwrap();
            if aabb.max.y < self.world_extents.bottom {
                remove_indices.push(index)
            }
        }

        let mut cnt = 0;
        for index in remove_indices {
            self.remove_body(commands, index-cnt);
            cnt+=1;
        }

        for (entity, body) in self.bodies.iter_mut() {
            let result = entity_q.get_mut(*entity);
            if let Ok((mut transform, _, _metadata)) = result {
                transform.translation.x = body.body.position.x;
                transform.translation.y = body.body.position.y;
                transform.rotation = Quat::from_rotation_z(body.body.rotation);
            }
        }
    }
}