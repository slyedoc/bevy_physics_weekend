#![allow(dead_code)]
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]

mod body;
mod bounds;
mod colliders;
mod constraints;
mod contact;
mod debug_render;
mod gjk;
mod intersect;
mod math;
mod broadphase;
mod narrowphase;
mod manifold;
mod dynamics;
mod resolve_contact;

use bevy::{ecs::schedule::ShouldRun, prelude::*};
use bevy_inspector_egui::Inspectable;

use bevy_polyline::*;
use body::Body;
use constraints::ConstraintPenetration;
use contact::{Contact, ContactMaybe};
use manifold::Manifold;
pub mod prelude {
    pub use crate::{
        body::*, colliders::*, constraints::*, contact::*, PhysicsConfig, PhysicsPlugin, debug_render::*,
    };
}

#[derive(Component, Inspectable)]
pub struct PhysicsConfig {
    pub enabled: bool,
    #[inspectable(min = -10.0, max = 10.0)]
    pub time_dilation: f32,
    pub gravity: Vec3,
    pub constrain_max_iter: usize, // solve constraints,
}

#[derive(Default)]
pub struct PhysicsTime {
    time: f32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            gravity: Vec3::new(0.0, -9.8, 0.0),
            constrain_max_iter: 5,
            time_dilation: 1.0,
        }
    }
}

/// The names of egui systems.
#[derive(SystemLabel, Clone, Hash, Debug, Eq, PartialEq)]
pub enum PhysicsSystem {
    First,
    Dynamics,
    Broadphase,
    Narrowphase,
    ConstraintsPreSolve,
    ConstraintsSolve,
    ResolveContact,
}

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(PolylinePlugin) // TODO: Make this optional
            .add_event::<Contact>()
            .add_event::<ContactMaybe>()
            .add_system_set(
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::First)
                    .with_system(timestep_system)
                    //.with_system(manifold_remove_expired_system),
            )
            .add_system_set(
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::Dynamics)
                    .after(PhysicsSystem::First)
                    .with_system(dynamics::dynamics_gravity_system),
            )
            .add_system_set(
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::Broadphase)
                    .after(PhysicsSystem::Dynamics)
                    .with_system(broadphase::broadphase_system),
            ).add_system_set(
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::Narrowphase)
                    .after(PhysicsSystem::Broadphase)
                    //.with_system(narrowphase::narrowphase_system),
                    .with_system(narrowphase::narrowphase_system_static),
            ).add_system_set(
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .after(PhysicsSystem::Broadphase)
                    .label(PhysicsSystem::ResolveContact)
                    .with_system(resolve_contact::resolve_contact_system),
            );
            // .add_system_set(
            //     SystemSet::new()
            //         .with_run_criteria(run_physics)
            //         .label(PhysicsSystem::ConstraintsPreSolve)
            //         .after(PhysicsSystem::Narrowphase)
            //         .with_system(constraints::constraint_penetration::pre_solve_system), //.with_system(constraints::post_solve_system::<ConstraintPenetration>)
            // )
            // .add_system_set(
            //     SystemSet::new()
            //         .with_run_criteria(run_physics)
            //         .label(PhysicsSystem::ConstraintsSolve)
            //         .after(PhysicsSystem::ConstraintsPreSolve)
            //         .with_system(constraints::constraint_penetration::solve_system),
            // );
            // // TODO: Make this optional
            // .add_system_set(
            //     SystemSet::new()
            //         .after(PhysicsSystem::ConstraintsSolve)
            //         .with_run_criteria(run_physics)
            //         .with_system(report_system),
            // )
            // // Debug Render
            app.add_system_set(
                SystemSet::new()
                    .after(PhysicsSystem::ResolveContact)
                    .with_system(debug_render::report_system),
            );


        // Add resources
        app.init_resource::<PhysicsConfig>();
        app.init_resource::<PhysicsTime>();

        // TODO: Move these to Components
        app.init_resource::<ContactVec>();
    }
}



fn run_physics(config: Res<PhysicsConfig>) -> ShouldRun {
    if config.time_dilation != 0.0 && config.enabled {
        ShouldRun::Yes
    } else {
        ShouldRun::No
    }
}

fn timestep_system(time: Res<Time>, config: Res<PhysicsConfig>, mut pt: ResMut<PhysicsTime>) {
    pt.time = time.delta_seconds() * config.time_dilation;
}


pub struct ContactVec(pub Vec<Contact>);

impl Default for ContactVec {
    fn default() -> Self {
        Self(Vec::<Contact>::with_capacity(50000))
    }
}
