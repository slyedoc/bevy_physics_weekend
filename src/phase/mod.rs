
mod broadphase;
mod narrowphase;
mod gravity;
mod ballistic_impulses;
mod constraints;

use bevy::prelude::Entity;
pub use broadphase::*;
pub use narrowphase::*;
pub use constraints::*;
pub use ballistic_impulses::*;
pub use gravity::*;

#[derive(Copy, Clone, Debug)]
pub struct CollisionPair {
    pub a: Entity,
    pub b: Entity,
}

impl PartialEq for CollisionPair {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for CollisionPair {}
