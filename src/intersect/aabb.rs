use crate::primitives::Aabb;


// Separating Axis Test
// If there is no overlap on a particular axis,
// then the two AABBs do not intersect
#[inline]
pub fn aabb_aabb_intersect(a: &Aabb, b: &Aabb) -> bool {
    if a.mins.x >= b.maxs.x {
        return false;
    }
    if a.maxs.x <= b.mins.x {
        return false;
    }

    if a.mins.y >= b.maxs.y {
        return false;
    }
    if a.maxs.y <= b.mins.y {
        return false;
    }

    if a.mins.z >= b.maxs.z {
        return false;
    }
    if a.maxs.z <= b.mins.z {
        return false;
    }

    // Overlap on all three axes, so their intersection must be non-empty
    true
}
