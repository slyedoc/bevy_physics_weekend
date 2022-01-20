use bevy::{math::Vec3, prelude::{Entity, Component}};

#[derive(Component, Copy, Clone, Debug)]
pub struct Contact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub world_point_a: Vec3,
    pub world_point_b: Vec3,
    pub local_point_a: Vec3,
    pub local_point_b: Vec3,
    pub normal: Vec3,
    pub separation_dist: f32,
    pub time_of_impact: f32,
}

impl Default for Contact {
    fn default() -> Self {
        Self {
            entity_a: Entity::from_raw(0),
            entity_b: Entity::from_raw(0),
            world_point_a: Default::default(),
            world_point_b: Default::default(),
            local_point_a: Default::default(),
            local_point_b: Default::default(),
            normal: Default::default(),
            separation_dist: Default::default(),
            time_of_impact: Default::default(),
        }
    }
}
