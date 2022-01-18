use bevy::prelude::*;

use crate::prelude::RigidBody;

use super::{Constraint, ConstraintConfig};

pub struct ConstraintMoverSimple {
    config: ConstraintConfig,
    time: f32,
}

impl ConstraintMoverSimple {
    pub fn new(config: ConstraintConfig) -> Self {
        ConstraintMoverSimple { config, time: 0.0 }
    }
}

impl Constraint for ConstraintMoverSimple {
    fn pre_solve(&mut self, bodies: &mut Query<(Entity, &mut RigidBody, &mut Transform)>, dt_sec: f32) {
        self.time += dt_sec;
        let (_, mut body_a, trans_a) = bodies.get_mut(self.config.handle_a.unwrap()).unwrap();

        body_a.linear_velocity.z = f32::cos(self.time * 0.25) * 4.0;
    }

    fn solve(&mut self, _bodies: &mut Query<(Entity, &mut RigidBody, &mut Transform)>) {}
}
