use bevy::prelude::*;

use crate::primitives::Body;


// TODO: currently we are just updating local from global, this is worng
// it breaks all nested transforms, but without a copy of global transform before our systems run
// not sure how you would do it
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
