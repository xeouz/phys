pub mod vector;
pub mod rigidbody;
pub mod world;
pub mod transform;
pub mod aabb;
pub mod manifold;
use bevy::prelude::{Vec2, Vec3};
use vector::PhysVector2;

use self::{rigidbody::{RigidBody, RigidBodyType}, vector::ZERO_VECTOR2, transform::PhysTransform, aabb::PhysAABB};

pub const SMALL_VAL: f32 = 0.0005;

pub fn clamp<T>(value: T, min: T, max: T) -> T where T: std::fmt::Display + std::cmp::PartialEq + std::cmp::PartialOrd  {
    if min == max { return min }

    if min > max {
        panic!("Clamp's minimum={} is greater than maximum={}. Cannot clamp!", min, max);
    }

    if value < min { 
        return min;
    }

    if value > max {
        return max;
    }

    value
}

pub fn nearly_equal(a: f32, b: f32) -> bool {
    (a-b).abs() < SMALL_VAL
}

pub fn vector_nearly_equal(a: &PhysVector2, b: &PhysVector2) -> bool {
    nearly_equal(a.x, b.x) && nearly_equal(a.y, b.y)
}

pub fn vector_average(a: &PhysVector2, b: &PhysVector2) -> PhysVector2 {
    a.add(b).div(2.0)
}

pub fn vector_length(a: &PhysVector2) -> f32 {
    f32::sqrt(a.x*a.x + a.y*a.y)
}

pub fn vector_length_squared(a: &PhysVector2) -> f32 {
    a.x*a.x + a.y*a.y
}

pub fn vector_distance(a: &PhysVector2, b: &PhysVector2) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    f32::sqrt(dx*dx + dy*dy)
}

pub fn vector_distance_squared(a: &PhysVector2, b: &PhysVector2) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx*dx + dy*dy
}

pub fn vector_normalize(a: &PhysVector2) -> PhysVector2 {
    let len = vector_length(a);
    PhysVector2 { x:a.x/len, y:a.y/len }
}

pub fn vector_dot(a: &PhysVector2, b: &PhysVector2) -> f32 {
    a.x*b.x + a.y*b.y
}

pub fn vector_cross(a: &PhysVector2, b: &PhysVector2) -> f32 {
    a.x*b.y - a.y*b.x
}

pub fn vector_floor(a: &PhysVector2) -> PhysVector2 {
    PhysVector2 { x: a.x.floor(), y: a.y.floor() }
}

pub fn vector_to_vec2(a: &PhysVector2) -> Vec2 {
    Vec2 { x: a.x, y: a.y }
}

pub fn vec2_to_vector(a: &Vec2) -> PhysVector2 {
    PhysVector2 { x: a.x, y: a.y }
}

pub fn vec3_to_vector(a: &Vec3) -> PhysVector2 {
    PhysVector2 { x: a.x, y: a.y }
}

pub fn simple_point_in_body(point: &PhysVector2, body: &RigidBody) -> bool {
    match &body.body_type {
        &RigidBodyType::Circle { radius } => {
            vector_distance(&body.position, point) <= radius
        },
        &RigidBodyType::Rect { width, height, data: _ } => {
            let (half_w, half_h) = (width/2.0, height/2.0);
            (point.x <= body.position.x + half_w && body.position.x - half_w <= point.x)
            &&
            (point.y >= body.position.y - half_h && body.position.y + half_h >= point.y)
        },
    }
}

pub fn vector_transform(vec: PhysVector2, transform: PhysTransform) -> PhysVector2 {
    let rx = (transform.comp_cos * vec.x) - (transform.comp_sin * vec.y);
    let ry = (transform.comp_sin * vec.x) + (transform.comp_cos * vec.y);
    
    let tx = rx + transform.position.x;
    let ty = ry + transform.position.y;

    PhysVector2 { x: tx, y: ty }
}

pub fn vertices_arithmetic_mean(vertices: &Vec<PhysVector2>) -> PhysVector2 {
    let (mut sum_x, mut sum_y) = (0.0,0.0);

    for vertex in vertices {
        sum_x += vertex.x;
        sum_y += vertex.y;
    };

    PhysVector2 { x: sum_x/(vertices.len() as f32), y: sum_y/(vertices.len() as f32) }
}

fn point_segment_distance_squared(point: &PhysVector2, a: &PhysVector2, b: &PhysVector2) -> (f32, PhysVector2) {
    let (mut dist, mut contact) = (0.0, ZERO_VECTOR2);

    let ab = b.sub(a);
    let ap = point.sub(a);

    let proj = vector_dot(&ap, &ab);
    let ab_len_sq = vector_length_squared(&ab);
    let d = proj / ab_len_sq;

    if d <= 0.0 {
        contact = *a;
    }
    else if d >= 1.0 {
        contact = *b;
    }
    else {
        contact = ab.mul(d).add(a);
    }

    dist = vector_distance_squared(point, &contact);

    (dist, contact)
}

fn project_vertices(axis: &PhysVector2, vertices: &Vec<PhysVector2>) -> (f32, f32) {
    let (mut min, mut max) = (f32::MAX, f32::MIN);

    for vertex in vertices {
        let proj = vector_dot(vertex, &axis);
        if proj < min {
            min = proj;
        }
        if proj > max {
            max = proj;
        }
    };

    (min, max)
}

fn project_circle(center: &PhysVector2, radius: f32, axis: &PhysVector2) -> (f32, f32) {
    let direction = vector_normalize(axis);
    let direction_and_radius = direction.mul(radius);

    let point1 = center.add(&direction_and_radius);
    let point2 = center.sub(&direction_and_radius);

    let a = vector_dot(&point1, axis);
    let b = vector_dot(&point2, axis);

    let (min, max) = if a < b {
        (a, b)
    }
    else {
        (b, a)
    };

    (min, max)
}

fn closest_point<'a>(center: &PhysVector2, vertices: &'a Vec<PhysVector2>) -> &'a PhysVector2 {
    let mut result: &PhysVector2 = &vertices[0];
    let mut min_distance = f32::MAX;

    for vertex in vertices {
        let distance = vector_distance(vertex, center);
        if distance < min_distance {
            result = vertex;
            min_distance = distance;
        }
    };

    result
}

pub fn intersect_circle(center_a: &PhysVector2, radius_a: f32, center_b: &PhysVector2, radius_b: f32) -> (bool, PhysVector2, f32) {
    let mut normal = ZERO_VECTOR2;
    let mut depth = 0.0;

    let distance = vector_distance(&center_a, &center_b);
    let radii = radius_a + radius_b;

    if distance >= radii {
        return (false, normal, depth)
    };

    normal = vector_normalize(&center_b.sub(&center_a));
    depth = radii - distance;

    return (true, normal, depth);
}

pub fn intersect_polygons_centered(vertices_a: &Vec<PhysVector2>, vertices_b: &Vec<PhysVector2>, center_a: PhysVector2, center_b: PhysVector2) -> (bool, PhysVector2, f32) {
    let mut normal = ZERO_VECTOR2;
    let mut depth = f32::MAX;

    for i in 0..vertices_a.len() {
        let va = vertices_a[i];
        let vb = vertices_a[(i+1)%vertices_a.len()];

        let edge = vb.sub(&va);
        let axis = vector_normalize(&PhysVector2 { x: -edge.y, y: edge.x });

        let (min_a, max_a) = project_vertices(&axis, vertices_a);
        let (min_b, max_b) = project_vertices(&axis, vertices_b);

        if min_a >= max_b || min_b >= max_a {
            return (false, normal, depth);
        }

        let axis_depth = f32::min(max_b-min_a, max_a-min_b);
        if axis_depth < depth {
            depth = axis_depth;
            normal = axis;
        }
    };

    for i in 0..vertices_b.len() {
        let va = vertices_b[i];
        let vb = vertices_b[(i+1)%vertices_b.len()];

        let edge = vb.sub(&va);
        let axis = vector_normalize(&PhysVector2 { x: -edge.y, y: edge.x });

        let (min_a, max_a) = project_vertices(&axis, vertices_a);
        let (min_b, max_b) = project_vertices(&axis, vertices_b);

        if min_a >= max_b || min_b >= max_a {
            return (false, normal, depth);
        }

        let axis_depth = f32::min(max_b-min_a, max_a-min_b);
        if axis_depth < depth {
            depth = axis_depth;
            normal = axis;
        }
    };

    let direction = center_b.sub(&center_a);

    if vector_dot(&direction, &normal) < 0.0 {
        normal = normal.neg();
    }

    (true, normal, depth)
}

pub fn intersect_polygons(vertices_a: &Vec<PhysVector2>, vertices_b: &Vec<PhysVector2>) -> (bool, PhysVector2, f32) {
    let center_a = vertices_arithmetic_mean(vertices_a);
    let center_b = vertices_arithmetic_mean(vertices_b);
    intersect_polygons_centered(vertices_a, vertices_b, center_a, center_b)
}

pub fn intersect_circle_polygon_centered(center: &PhysVector2, radius: f32, vertices: &Vec<PhysVector2>, polygon_center: PhysVector2) -> (bool, PhysVector2, f32) {
    let mut normal = ZERO_VECTOR2;
    let mut depth = f32::MAX;
    
    for i in 0..vertices.len() {
        let va = vertices[i];
        let vb = vertices[(i+1)%vertices.len()];

        let edge = vb.sub(&va);
        let axis = vector_normalize(&PhysVector2 { x: -edge.y, y: edge.x });

        let (min_a, max_a) = project_vertices(&axis, vertices);
        let (min_b, max_b) = project_circle(center, radius, &axis);

        if min_a >= max_b || min_b >= max_a {
            return (false, normal, depth);
        }

        let axis_depth = f32::min(max_b-min_a, max_a-min_b);
        if axis_depth < depth {
            depth = axis_depth;
            normal = axis;
        }
    };

    let closest = closest_point(center, vertices);
    let axis = vector_normalize(&closest.sub(center));

    let (min_a, max_a) = project_vertices(&axis, vertices);
    let (min_b, max_b) = project_circle(center, radius, &axis);

    if min_a >= max_b || min_b >= max_a {
        return (false, normal, depth);
    }

    let axis_depth = f32::min(max_b-min_a, max_a-min_b);
    if axis_depth < depth {
        depth = axis_depth;
        normal = axis;
    };

    let direction = polygon_center.sub(&center);

    if vector_dot(&direction, &normal) < 0.0 {
        normal = normal.neg();
    }

    (true, normal, depth)
}

pub fn intersect_circle_polygon(center: &PhysVector2, radius: f32, vertices: &Vec<PhysVector2>) -> (bool, PhysVector2, f32) {
    let polygon_center = vertices_arithmetic_mean(vertices);
    intersect_circle_polygon_centered(center, radius, vertices, polygon_center)
}

pub fn intersect_aabb(a: &PhysAABB, b: &PhysAABB) -> bool {
    if a.max.x <= b.min.x || b.max.x <= a.min.x
    || a.max.y <= b.min.y || b.max.y <= a.min.y {
        false
    }
    else {
        true
    }
}

fn find_contact_point_circle(center_a: &PhysVector2, radius_a: f32, center_b: &PhysVector2) -> PhysVector2 {
    let ab = center_b.sub(center_a);
    let dir = vector_normalize(&ab);
    
    let cp = dir.mul(radius_a).add(center_a);

    cp
}

fn find_contact_point_circle_polygon(center: &PhysVector2, radius: f32, vertices: &Vec<PhysVector2>) -> PhysVector2 {
    let mut min_dist = f32::MAX;
    let mut cp = ZERO_VECTOR2;

    for i in 0..vertices.len() {
        let (va, vb) = (vertices[i], vertices[(i+1)%vertices.len()]);
        let (dist_sq, contact) = point_segment_distance_squared(center, &va, &vb);

        if dist_sq < min_dist {
            min_dist = dist_sq;
            cp = contact;
        }
    }

    cp
}

fn find_contact_point_polygon(vertices_a: &Vec<PhysVector2>, vertices_b: &Vec<PhysVector2>) -> (PhysVector2, PhysVector2, u8) {
    let (mut contact_one, mut contact_two, mut contact_count) = (ZERO_VECTOR2, ZERO_VECTOR2, 0);
    let mut min_dist = f32::MAX;

    for p in vertices_a {
        for i in 0..vertices_b.len() {
            let va = vertices_b[(i)];
            let vb = vertices_b[(i+1)%vertices_b.len()];

            let (dist_sq, cp) = point_segment_distance_squared(p, &va, &vb);
            
            if nearly_equal(dist_sq, min_dist) {
                if !vector_nearly_equal(&cp, &contact_one) {
                    contact_two = cp;
                    contact_count = 2;
                }
            }
            else if dist_sq < min_dist {
                min_dist = dist_sq;
                contact_count = 1;
                contact_one = cp;
            }
        }
    }

    for p in vertices_b {
        for i in 0..vertices_a.len() {
            let va = vertices_a[(i)];
            let vb = vertices_a[(i+1)%vertices_a.len()];

            let (dist_sq, cp) = point_segment_distance_squared(p, &va, &vb);
            
            if nearly_equal(dist_sq, min_dist) {
                if !vector_nearly_equal(&cp, &contact_one) {
                    contact_two = cp;
                    contact_count = 2;
                }
            }
            else if dist_sq < min_dist {
                min_dist = dist_sq;
                contact_count = 1;
                contact_one = cp;
            }
        }
    }

    (contact_one, contact_two, contact_count)
}

pub fn find_contact_points(body_a: &mut RigidBody, body_b: &mut RigidBody) -> (PhysVector2, PhysVector2, u8) {
    let (mut contact_one, mut contact_two, mut contact_count) = (ZERO_VECTOR2, ZERO_VECTOR2, 0 as u8);
    if body_a.get_body_index() == body_b.get_body_index()
    {
        if body_a.get_body_index() == RigidBodyType::RECT_INDEX { // RECT TO RECT
            (contact_one, contact_two, contact_count) = find_contact_point_polygon(
                body_a.get_transformed_vertices().unwrap(), 
                body_b.get_transformed_vertices().unwrap()
            );
        }
        
        else if body_a.get_body_index() == RigidBodyType::CIRCLE_INDEX { // CIRCLE TO CIRCLE
            let radius = match body_a.body_type { RigidBodyType::Circle { radius } => radius, _ => 0.0 };
            contact_one = find_contact_point_circle(&body_a.position, radius, &body_b.position);
            contact_count = 1;
        }
    }
    else {
        if body_a.get_body_index() == RigidBodyType::RECT_INDEX
        && body_b.get_body_index() == RigidBodyType::CIRCLE_INDEX { // RECT TO CIRCLE
            let radius = match body_b.body_type { RigidBodyType::Circle { radius } => radius, _ => 0.0 };
            contact_one = find_contact_point_circle_polygon(&body_b.position, radius, body_a.get_transformed_vertices().unwrap());
            contact_count = 1;
        }
        
        else if body_a.get_body_index() == RigidBodyType::CIRCLE_INDEX
        && body_b.get_body_index() == RigidBodyType::RECT_INDEX { // CIRCLE TO RECT
            let radius = match body_a.body_type { RigidBodyType::Circle { radius } => radius, _ => 0.0 };
            contact_one = find_contact_point_circle_polygon(&body_a.position, radius, body_b.get_transformed_vertices().unwrap());
            contact_count = 1;
        }
    }

    (contact_one, contact_two, contact_count)
}