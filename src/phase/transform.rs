use bevy::prelude::*;

use crate::primitives::Body;

pub fn update_local_tranform(
    mut query: Query<(&GlobalTransform, &mut Transform), With<Body>>,
) {
    for (gt, mut t) in query.iter_mut() {
        t.translation = gt.translation;
        t.rotation = gt.rotation;
        // t.set_rotation_xyz(gt.rotation.x, gt.rotation.y, gt.rotation.z);
        // t.set_scale(gt.scale);
    }
}
