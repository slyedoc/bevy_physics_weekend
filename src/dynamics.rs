use bevy::prelude::*;
use crate::{PhysicsConfig, body::Body, PhysicsTime};

pub fn dynamics_gravity_system(
    config: Res<PhysicsConfig>,
    pt: Res<PhysicsTime>,
    mut query: Query<&mut Body>,
) {
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
}
