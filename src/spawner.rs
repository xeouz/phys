use std::fmt;

use bevy::{prelude::*, ecs::system::EntityCommands};
use bevy_prototype_lyon::{shapes, prelude::{ShapeBundle, GeometryBuilder, Fill, Stroke, tess::geom::Translation}};
use once_cell::sync::Lazy;

use crate::math::{vector::PhysVector2, vector_to_vec2, vector_distance};

pub static DEFAULT_Z_VALUE: f32 = 1.0;
pub static DEFAULT_STROKE_WIDTH: f32 = 2.5;
pub static DEFAULT_FILL_COLOR: Lazy<Color> = Lazy::new(|| {
    Color::rgba_u8(224, 108, 117, 255)
});
pub static DEFAULT_STROKE_COLOR: Lazy<Color> = Lazy::new(|| {
    Color::rgba_u8(20, 24, 32, 255)
    // Color::rgba_u8(0, 0, 0, 0)
});

#[derive(Debug)]
pub struct SpawnError;

#[derive(Component, PartialEq)]
pub enum SpawnType {
    Rectangle { height: f32, width: f32 },
    Circle { radius: f32 },
    Line,
}

#[derive(Component)]
pub struct SpawnMetadata {
    pub shape_type: SpawnType,
}

pub struct SpawnArgs {
    pub z_value: f32,
    pub fill_color: Color,
    pub stroke_color: Color,
    pub stroke_width: f32,
    pub is_visible: bool,
}

impl Default for SpawnArgs {
    fn default() -> Self {
        Self { z_value: DEFAULT_Z_VALUE, fill_color: *DEFAULT_FILL_COLOR, stroke_color: *DEFAULT_STROKE_COLOR, stroke_width: DEFAULT_STROKE_WIDTH, is_visible: true }
    }
}

pub fn spawn_line(commands: &mut Commands, start: &PhysVector2, end: &PhysVector2, args: SpawnArgs) -> Result<Entity, SpawnError> {
    let shape = shapes::Line {
        0: Vec2::ZERO,
        1: vector_to_vec2(&end.sub(start))
    };

    let entity = commands.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&shape),
            transform: Transform::from_xyz(start.x, start.y, args.z_value),
            visibility: if args.is_visible { Visibility::Visible } else { Visibility::Hidden },
            ..default()
        },
        Fill::color(args.fill_color),
        Stroke::new(args.stroke_color, args.stroke_width),
        SpawnMetadata { shape_type: SpawnType::Line },
    )).id();

    Ok(entity)
}

pub fn spawn_circle(commands: &mut Commands, position: &PhysVector2, radius: f32, args: SpawnArgs) -> Result<Entity, SpawnError> {
    let mut shape = shapes::Circle::default();
    shape.center = Vec2::ZERO;
    shape.radius = radius;

    let entity = commands.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&shape),
            transform: Transform::from_xyz(position.x, position.y, args.z_value),
            visibility: if args.is_visible { Visibility::Visible } else { Visibility::Hidden },
            ..default()
        },
        Fill::color(args.fill_color),
        Stroke::new(args.stroke_color, args.stroke_width),
        SpawnMetadata { shape_type: SpawnType::Circle { radius: radius } },
    )).id();

    
    Ok(entity)
}

pub fn spawn_rect(commands: &mut Commands, position: &PhysVector2, height: f32, width: f32, args: SpawnArgs) -> Result<Entity, SpawnError> {
    let mut shape = shapes::Rectangle::default();
    shape.origin = shapes::RectangleOrigin::Center;
    shape.extents = Vec2 { x: width, y: height };

    let entity = commands.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&shape),
            transform: Transform::from_xyz(position.x, position.y, args.z_value),
            visibility: if args.is_visible { Visibility::Visible } else { Visibility::Hidden },
            ..default()
        },
        Fill::color(args.fill_color),
        Stroke::new(args.stroke_color, args.stroke_width),
        SpawnMetadata { shape_type: SpawnType::Rectangle { height: height, width: width } },
    )).id();

    Ok(entity)
}

pub fn spawn_line_default(commands: &mut Commands, start: &PhysVector2, end: &PhysVector2) -> Result<Entity, SpawnError> {
    spawn_line(commands, start, end, SpawnArgs::default())
}

pub fn spawn_circle_default(commands: &mut Commands, position: &PhysVector2, radius: f32) -> Result<Entity, SpawnError> {
    spawn_circle(commands, &position, radius, SpawnArgs::default())
}

pub fn spawn_rect_default(commands: &mut Commands, position: &PhysVector2, height: f32, width: f32) -> Result<Entity, SpawnError> {
    spawn_rect(commands, &position, height, width, SpawnArgs::default())
}