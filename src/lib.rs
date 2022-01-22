#![allow(dead_code)]
#![allow(clippy::type_complexity)]
mod body;
mod bounds;
mod colliders;
mod constraints;
mod contact;
mod gjk;
mod intersect;
mod math;
mod phase;

use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

use constraints::ConstraintPenetration;
use phase::*;
use prelude::{ConstraintArena, Contact};
use bevy_polyline::*;

pub mod prelude {
    pub use crate::{
        body::*,
        colliders::*,
        constraints::*,
        contact::*,
        phase::*,
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
    Dynamics,
    Broadphase,
    Narrowphase,
    AddManifold,
    Constraints,
    ApplyBallisticImpulses,
}

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {

        app
            .add_plugin(PolylinePlugin) // TODO: MOve this to debug render plugin
            .add_event::<Contact>()
            .add_system_set(
                SystemSet::new()
                    .label(PhysicsSystem::PreUpdate)
                    .with_system(timestep_system)
                    .with_system(manifold_remove_expired_system),
            )
            .add_system(
                    dynamics_gravity_system
                    .label(PhysicsSystem::Dynamics)
                    .after(PhysicsSystem::PreUpdate),
            )
            .add_system(
                broadphase_system
                    .label(PhysicsSystem::Broadphase)
                    .after(PhysicsSystem::Dynamics),
            )
            .add_system(
                narrowphase_system
                    .label(PhysicsSystem::Narrowphase)
                    .after(PhysicsSystem::Broadphase),
            )
            .add_system_set(
                SystemSet::new()
                    .label(PhysicsSystem::Constraints)
                    .after(PhysicsSystem::Narrowphase)
                    .with_system(constraints::pre_solve_system::<ConstraintPenetration>)
                    .with_system(constraints::solve_system::<ConstraintPenetration>)
                    .with_system(constraints::post_solve_system::<ConstraintPenetration>)
                    .with_system(constraints_system),
                    
            )
            .add_system(
                ballistic_impulses_system
                    .label(PhysicsSystem::Constraints)
                    .after(PhysicsSystem::Narrowphase),
            );

        // Add resources
        app.init_resource::<PhysicsConfig>();
        app.init_resource::<PhysicsTime>();
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
        Self(Vec::<CollisionPair>::with_capacity(100000))
    }
}

pub struct ContactVec(pub Vec<Contact>);

impl Default for ContactVec {
    fn default() -> Self {
        Self(Vec::<Contact>::with_capacity(50000))
    }
}
