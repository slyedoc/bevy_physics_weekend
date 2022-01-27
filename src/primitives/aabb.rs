use bevy::{prelude::Component, math::Vec3};


// Axis-Aligned Bounding Box
#[derive(Component)]
pub struct Aabb {
    pub mins: Vec3,
    pub maxs: Vec3,
}
