use rand::Rng;
use std::{mem::discriminant, borrow::BorrowMut, f32::consts::PI};

use bevy::{prelude::*, ecs::system::Command, app::AppExit};
use bevy_prototype_lyon::prelude::*;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};

mod spawner;
mod math;
mod manager;
use manager::{PhysManager, WorldData, PhysBody};
use math::{vector::{PhysVector2, ZERO_VECTOR2}, rigidbody::{create_circle_body, create_rect_body, RigidBodyType}, simple_point_in_body, vec2_to_vector, intersect_circle, vector_to_vec2, vec3_to_vector, vector_normalize, vector_distance};
use spawner::{SpawnMetadata, DEFAULT_STROKE_COLOR, SpawnArgs, DEFAULT_STROKE_WIDTH};

use crate::{spawner::SpawnType, math::vector_floor};

#[derive(Component)]
pub struct CursorImpulseShape;

fn main() {
    App::new()
        .insert_resource(Msaa::Sample4)
        .insert_resource(PhysManager::default())
        .insert_resource(WorldData::default())
        .insert_resource(ClearColor(Color::rgba_u8(40, 44, 52, 255)))
        .add_plugins(DefaultPlugins)
        .add_plugins(ShapePlugin)
        .add_plugins(LogDiagnosticsPlugin::default())
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, handle_update)
        .add_systems(Update, handle_keyboard)
        .add_systems(Update, handle_mouse)
        .run();
}

fn setup(
    mut commands: Commands,
    mut manager: ResMut<PhysManager>,
) {
    let shape = shapes::Line {
        0: Vec2::ZERO,
        1: Vec2::ZERO
    };
    commands.spawn(Camera2dBundle::default());
    commands.spawn((ShapeBundle {
        path: GeometryBuilder::build_as(&shape),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
        },
        Fill::color(Color::CYAN),
        Stroke::new(Color::GRAY, 3.0),
        CursorImpulseShape
    ));
    
    let bodies = vec![
        (Some(SpawnArgs { z_value: 1.0, fill_color: Color::GRAY, stroke_color: Color::DARK_GRAY, stroke_width: 3.0, is_visible: true }),
         create_rect_body(PhysVector2 { x: 0.0, y: -70.0 }, 2.0, 0.5, 70.0, 700.0, true)),
    ];
    manager.add_bodies(&mut commands, bodies).unwrap();
}

fn handle_update(
    time: Res<Time>,
    mut commands: Commands,
    mut manager: ResMut<PhysManager>,
    mut entity_q: Query<(&mut Transform, &mut Visibility, &mut SpawnMetadata)>,
) {
    use std::time::Instant;
    let now = Instant::now();
    manager.step(&mut commands, entity_q, time.raw_delta_seconds(), 30);
    let elapsed = now.elapsed();
    println!("Bodies: {}, Elapsed: {:.2?}", manager.bodies.len(), elapsed.as_millis());
}

fn handle_keyboard(
    input: Res<Input<KeyCode>>,
    windows: Query<&Window>,
    mut commands: Commands,
    mut manager: ResMut<PhysManager>,
    mut app_exit_events: ResMut<Events<AppExit>>,
) {
    let res = windows.get_single();
    if res.is_err() { return; }
    let window = res.unwrap();
    let pos = window.cursor_position();
    if pos.is_none() { return; }

    let mut point = vec2_to_vector(&pos.unwrap());
    point.x -= window.width() / 2.0;
    point.y = (point.y - window.height() / 2.0) * -1.0;

    if input.pressed(KeyCode::Escape) {
        app_exit_events.send(AppExit)
    }
    else if input.just_pressed(KeyCode::E) {
        let (h,w) = (rand::thread_rng().gen_range(30..60) as f32, rand::thread_rng().gen_range(30..60) as f32);
        let (r,g, b) = (rand::thread_rng().gen_range(30..230), rand::thread_rng().gen_range(30..230), rand::thread_rng().gen_range(30..230));
        let body = create_rect_body(point, 2.0, 0.6, h, w, false);
        let color = Color::rgb_u8(r, g, b);
        let spawn_args = SpawnArgs { z_value: 1.0, fill_color: color, stroke_color: *DEFAULT_STROKE_COLOR, stroke_width: DEFAULT_STROKE_WIDTH, is_visible: true };
        manager.add_body(&mut commands, body, spawn_args).unwrap();
    }
    else if input.just_pressed(KeyCode::T) {
        let radius = rand::thread_rng().gen_range(15..30) as f32;
        let (r,g, b) = (rand::thread_rng().gen_range(30..230), rand::thread_rng().gen_range(30..230), rand::thread_rng().gen_range(30..230));
        let body = create_circle_body(point, 2.0, 0.6, radius, false);
        let color = Color::rgb_u8(r, g, b);
        let spawn_args = SpawnArgs { z_value: 1.0, fill_color: color, stroke_color: *DEFAULT_STROKE_COLOR, stroke_width: DEFAULT_STROKE_WIDTH, is_visible: true };
        manager.add_body(&mut commands, body, spawn_args).unwrap();
    }
}

fn handle_mouse(
    mut commands: Commands,
    mut entity_q: Query<(&mut Transform, &mut SpawnMetadata, &mut Stroke)>,
    mut impulse_q: Query<(&mut Path, &CursorImpulseShape)>,
    windows: Query<&Window>,
    input: Res<Input<MouseButton>>,
    mut manager: ResMut<PhysManager>,
    mut world: ResMut<WorldData>,
    time: Res<Time>
) {
    let res = windows.get_single();
    if res.is_err() { return; }
    let window = res.unwrap();
    let pos = window.cursor_position();
    if pos.is_none() { return; }

    let mut point = vec2_to_vector(&pos.unwrap());
    point.x -= window.width() / 2.0;
    point.y = (point.y - window.height() / 2.0) * -1.0;

    if input.just_pressed(MouseButton::Left) || input.just_pressed(MouseButton::Right) {
        for (index, (_entity, body)) in manager.bodies.iter().enumerate() {
            
            if simple_point_in_body(&point, &body.body) {
                world.select(body.entity, index);
                break
            }
        }

        if input.just_pressed(MouseButton::Right) && world.selected_body.is_some() {
            // 171, 178, 191
            let (entity, _) = world.selected_body.unwrap();
            let entity_result = entity_q.get_mut(entity);
            if let Ok((_, _, mut stroke)) = entity_result {
                stroke.color = Color::rgba_u8(136, 156, 194, 255);
            }
        }
    }

    if input.pressed(MouseButton::Left) && world.selected_body.is_some() {
        let (entity, index) = world.selected_body.unwrap();
        let result = entity_q.get_mut(entity);

        let phys_body_opt = manager.get_body_mut(index);
        
        if let Some((_, phys_body)) = phys_body_opt {
            if let Ok((mut _transform, metadata, _)) = result {
                if metadata.shape_type != SpawnType::Line {
                    let (padding_x, padding_y) = match &metadata.shape_type {
                        &SpawnType::Circle { radius: _ } => {
                            (0.0, 0.0)
                        }
                        &SpawnType::Rectangle { height: _, width: _ } => {
                            (0.0, 0.0)
                        }
                        &SpawnType::Line => {
                            (0.0, 0.0)
                        }
                    };

                    let (final_x, final_y) = (point.x + padding_x, point.y + padding_y);
                    let original_position = phys_body.body.position;
                    let final_position = PhysVector2 { x: final_x, y: final_y };
                    phys_body.move_to_body(&final_position).unwrap();
                    
                    let diff = PhysVector2 { x: final_position.x - original_position.x, y: final_position.y - original_position.y };
                    let force = vector_floor(&diff).mul(5.0);
                    phys_body.body.linear_velocity = force;
                }
            }
        }
    }
    else if input.pressed(MouseButton::Right) && world.selected_body.is_some() {
        let (entity, _) = world.selected_body.unwrap();
        let entity_result = entity_q.get_mut(entity);
        if let Ok((transform, _, _)) = entity_result {
            let result = impulse_q.get_single_mut();
            if let Ok((mut path, _)) = result {
                let start = vector_to_vec2(&point);
                let end = Vec2 { x: transform.translation.x, y: transform.translation.y };
                let shape = shapes::Line {
                    0: start,
                    1: end
                };
                *path = ShapePath::build_as(&shape);
            }
        }
    }

    if input.just_released(MouseButton::Left) {
        world.selected_body = None;
    }

    if input.just_released(MouseButton::Right) && world.selected_body.is_some() {
        let (entity, index) = world.selected_body.unwrap();
        let entity_result = entity_q.get_mut(entity);
        if let Ok((transform, _, mut stroke)) = entity_result {
            stroke.color = *DEFAULT_STROKE_COLOR;

            let impulse = point.sub(&vec3_to_vector(&transform.translation)).neg();
            let (_, body) = manager.get_body_mut(index).unwrap();
            body.add_force(impulse);

            let result = impulse_q.get_single_mut();
            if let Ok((mut path, _)) = result {
                let shape = shapes::Line {
                    0: Vec2::ZERO,
                    1: Vec2::ZERO
                };
                *path = ShapePath::build_as(&shape);
            }
        }
    }
}