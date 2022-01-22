use bevy::prelude::*;
use crate::{PhysicsConfig, prelude::Body, PhysicsTime};

pub fn dynamics_gravity_system(
    //pool: Res<ComputeTaskPool>,
    config: Res<PhysicsConfig>,
    pt: Res<PhysicsTime>,
    mut query: Query<&mut Body>,
) {
    if pt.time == 0.0 || config.gravity == Vec3::ZERO {
        return;
    }

    for mut body in query.iter_mut() {
        if body.inv_mass != 0.0 {
            // gravity needs to be an impulse
            // I = dp, F = dp/dt => dp = F * dt => I = F * dt
            // F = mgs
            let mass = 1.0 / body.inv_mass;
            let impluse_gravity = config.gravity * mass * pt.time;
            body.apply_impulse_linear(impluse_gravity);
        }
    }

    // TODO: Test task pool
    // query.par_for_each_mut(&pool, 32, |mut body| {
    //     if body.inv_mass != 0.0 {
    //         // gravity needs to be an impulse
    //         // I = dp, F = dp/dt => dp = F * dt => I = F * dt
    //         // F = mgs
    //         let mass = 1.0 / body.inv_mass;
    //         let impluse_gravity = config.gravity * mass * dt;
    //         body.apply_impulse_linear(impluse_gravity);
    //     }
    // });
}
