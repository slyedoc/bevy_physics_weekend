#![allow(dead_code)]
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]
#![feature(vec_retain_mut)]
#![feature(div_duration)]

pub mod constraints;
pub mod debug_render;
mod intersect;
mod math;
mod phase;
pub mod primitives;

use primitives::*;

use bevy::{ecs::schedule::ShouldRun, prelude::*};
use bevy_inspector_egui::Inspectable;
use phase::*;

use bevy_polyline::*;


#[derive(Component, Inspectable)]
pub struct PhysicsConfig {
    pub enabled: bool,
    #[inspectable(min = -10.0, max = 10.0)]
    pub time_dilation: f32,
    pub gravity: Vec3,
    pub constrain_max_iter: usize,
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
    NarrowphaseBreakout,
    Narrowphase,
    ConstraintsPreSolve,
    ConstraintsSolve,
    ResolveContact,
}


pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<Contact>()
            .add_event::<BroadContact>()
            .add_event::<BroadSphereSphere>()
            .add_event::<BroadSphereBox>()
            .add_event::<BroadBoxBox>()
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::First)
                    .with_system(timestep_system)
                    .with_system(update_aabb::<primitives::ColliderSphere>)
                    .with_system(update_aabb::<primitives::ColliderBox>)
                    .with_system(collider_add_watch::<primitives::ColliderSphere>)
                    .with_system(collider_add_watch::<primitives::ColliderBox>),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::Dynamics)
                    .after(PhysicsSystem::First) // chaining
                    .with_system(dynamics::dynamics_gravity_system),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::Broadphase)
                    .after(PhysicsSystem::Dynamics) // chaining
                    //.with_system(broadphase::broadphase_system)
                    //.with_system(broadphase::broadphase_system_aabb)
                    .with_system(broadphase::broadphase_system),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::NarrowphaseBreakout)
                    .after(PhysicsSystem::Broadphase) // chaining
                    //.with_system(narrowphase::narrowphase_system),
                    .with_system(narrowphase::narrowphase_breakout_system),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::Narrowphase)
                    .after(PhysicsSystem::NarrowphaseBreakout) // chaining
                    .with_system(narrowphase::narrow_phase_sphere_box)
                    .with_system(narrowphase::narrow_phase_sphere_sphere),
            )
            .add_system_set_to_stage(
                CoreStage::PreUpdate,
                SystemSet::new()
                    .with_run_criteria(run_physics)
                    .label(PhysicsSystem::ResolveContact) // chaining
                    .after(PhysicsSystem::Narrowphase)
                    .with_system(resolve_contact::resolve_contact_system),
            );

        //.with_system(manifold_remove_expired_system),

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

        // Debug Render
        // TODO: Make this optional
        app.add_plugin(PolylinePlugin).add_system_set_to_stage(
            CoreStage::PreUpdate,
            SystemSet::new()
                .after(PhysicsSystem::ResolveContact)
                .with_system(debug_render::report_system),
        );

        // Add resources
        app.init_resource::<PhysicsConfig>();
        app.init_resource::<PhysicsTime>();
    }
}

pub fn collider_add_watch<T: Component + Collider>(mut commands: Commands, query: Query<(Entity, &GlobalTransform, &T), Added<T>>) {
    for (e, trans, collider) in query.iter() {
        commands.entity(e).insert(collider.aabb(trans));
        commands.entity(e).insert(collider.shape_type());
    }
}

pub fn update_aabb<T: Component + Collider>(
    mut query: Query<(&GlobalTransform, &T, &mut Aabb)>,
) {
    for (trans, collider, mut aabb) in query.iter_mut() {
        let new_aabb = collider.aabb(trans);
        aabb.mins = new_aabb.mins;
        aabb.maxs = new_aabb.maxs;

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
