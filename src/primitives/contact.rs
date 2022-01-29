use bevy::{math::Vec3, prelude::{Entity}};

#[derive( Copy, Clone, Debug)]
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

#[derive(Copy, Clone, Debug)]
pub struct BroadContact {
    pub a: Entity,
    pub b: Entity,
}


impl PartialEq for BroadContact {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for BroadContact {}

#[derive(Copy, Clone, Debug)]
pub struct PsuedoBody {
    pub entity: Entity,
    pub value: f32,
    pub is_min: bool,
}

impl PsuedoBody {
    pub fn compare_sat(a: &PsuedoBody, b: &PsuedoBody) -> std::cmp::Ordering {
        if a.value < b.value {
            std::cmp::Ordering::Less
        } else if a.value > b.value {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    }
}
