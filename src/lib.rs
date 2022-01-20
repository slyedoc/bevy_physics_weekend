#![allow(dead_code)]
#![allow(clippy::type_complexity)]
mod body;
mod bounds;
mod colliders;
mod constraints;
mod contact;
mod gjk;
mod intersect;
mod manifold;
mod math;
mod phase;

use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

use manifold::*;
use phase::*;
use prelude::{ConstraintArena, Contact};

pub mod prelude {
    pub use crate::{
        body::*,
        colliders::*,
        constraints::*,
        contact::*,
        manifold::*, // TODO: remove
        PhysicsConfig,
        PhysicsPlugin,
    };
}

#[derive(Component, Inspectable)]
pub struct PhysicsConfig {
    #[inspectable(min = -10.0, max = 10.0)]
    time_dilation: f32,
    gravity: Vec3,
    constrain_max_iter: usize, // solve constraints,
}

#[derive(Default)]
pub struct PhysicsTime {
    time: f32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.8, 0.0),
            constrain_max_iter: 5,
            time_dilation: 1.0,
        }
    }
}

/// The names of egui systems.
#[derive(SystemLabel, Clone, Hash, Debug, Eq, PartialEq)]
pub enum PhysicsSystem {
    PreUpdate,
    Gravity,
    Broadphase,
    Narrowphase,
    AddManifold,
    Constraints,
    ApplyBallisticImpulses,
}

pub struct AddManifold(pub Contact);

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<Contact>()
            .add_event::<AddManifold>()
            .add_system_set(
                SystemSet::new()
                    .label(PhysicsSystem::PreUpdate)
                    .with_system(timestep_system)
                    .with_system(manifold_remove_expired),
            )
            .add_system(
                gravity_system
                    .label(PhysicsSystem::Gravity)
                    .after(PhysicsSystem::PreUpdate),
            )
            .add_system(
                broadphase_system
                    .label(PhysicsSystem::Broadphase)
                    .after(PhysicsSystem::Gravity),
            )
            .add_system(
                narrowphase_system
                    .label(PhysicsSystem::Narrowphase)
                    .after(PhysicsSystem::Broadphase),
            )
            .add_system(
                manifold_add_system
                     .label(PhysicsSystem::AddManifold)
                     .after(PhysicsSystem::Narrowphase),
            )
            .add_system(
                constraints_system
                    .label(PhysicsSystem::Constraints)
                    .after(PhysicsSystem::AddManifold),
            )
            .add_system(
                ballistic_impulses_system
                    .label(PhysicsSystem::ApplyBallisticImpulses)
                    .after(PhysicsSystem::Narrowphase),
            );

        // Add resources
        app.init_resource::<PhysicsConfig>();
        app.init_resource::<PhysicsTime>();
        app.init_resource::<ManifoldCollector>();
        app.init_resource::<ConstraintArena>();
        app.init_resource::<CollisionPairVec>();
        app.init_resource::<ContactVec>();
    }
}

fn timestep_system(time: Res<Time>, config: Res<PhysicsConfig>, mut pt: ResMut<PhysicsTime>) {
    pt.time = time.delta_seconds() * config.time_dilation;
}


pub struct CollisionPairVec(pub Vec<CollisionPair>);

impl Default for CollisionPairVec {
    fn default() -> Self {
        Self(Vec::<CollisionPair>::with_capacity(1000))
    }
}

pub struct ContactVec(pub Vec<Contact>);

impl Default for ContactVec {
    fn default() -> Self {
        Self(Vec::<Contact>::with_capacity(50000))
    }
}
