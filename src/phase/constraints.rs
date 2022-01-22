use bevy::prelude::*;

use crate::{PhysicsConfig, body::Body, PhysicsTime};
use crate::constraints::Constraint;
use super::Manifold;

pub fn constraints_system(
    config: Res<PhysicsConfig>,
    pt: Res<PhysicsTime>,
    mut bodies: Query<(Entity, &mut Body, &mut Transform)>,
    mut manifolds: Query<&mut Manifold>,
) {
    // solve constraints
    for mut manifold in manifolds.iter_mut() {
        for cc in manifold.contact_contraints.iter_mut() {
            cc.constraint.pre_solve(&mut bodies, pt.time);
        }
    }

    for _ in 0..config.constrain_max_iter {
        //     constraints.solve(&mut query);
        for mut manifold in manifolds.iter_mut() {
            for cc in &mut manifold.contact_contraints {
                cc.constraint.solve(&mut bodies);
            }
        }
    }

    // constraints.post_solve();
    for mut manifold in manifolds.iter_mut() {
        for cc in &mut manifold.contact_contraints {
            cc.constraint.post_solve();
        }
    }
}
