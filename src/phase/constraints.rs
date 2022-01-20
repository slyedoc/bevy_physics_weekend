use bevy::prelude::*;

use crate::{PhysicsConfig, body::Body, prelude::Manifold, PhysicsTime};

pub fn constraints_system(
    config: Res<PhysicsConfig>,
    pt: Res<PhysicsTime>,
    mut query: Query<(Entity, &mut Body, &mut Transform)>,
    mut manifolds: Query<&mut Manifold>,
    //mut constraints: ResMut<ConstraintArena>,
) {
    // solve constraints
    // constraints.pre_solve(&mut query, dt);
    for mut manifold in manifolds.iter_mut() {
        manifold.pre_solve(&mut query, pt.time);
    }
    
    for _ in 0..config.constrain_max_iter {
        //     constraints.solve(&mut query);
        for mut manifold in manifolds.iter_mut() {
            manifold.solve(&mut query);
        }
    }

    // constraints.post_solve();
    for mut manifold in manifolds.iter_mut() {
        manifold.post_solve();
    }
}
