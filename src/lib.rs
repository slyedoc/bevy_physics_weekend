use bevy::prelude::*;

pub mod prelude {
    pub use crate::{
        PhysicsPlugin,
    };
}

/// The names of egui systems.
#[derive(SystemLabel, Clone, Hash, Debug, Eq, PartialEq)]
pub enum PhysicsSystem {
    Update
}

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {

    }
}