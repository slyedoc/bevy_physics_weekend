use crate::bounds::aabb::Aabb;


// Separating Axis Test
// If there is no overlap on a particular axis,
// then the two AABBs do not intersect
#[inline]
pub fn aabb_aabb_intersect(a: &Aabb, b: &Aabb) -> bool {
    if a.minimums().x >= b.maximums().x {
        return false;
    }
    if a.maximums().x <= b.minimums().x {
        return false;
    }

    if a.minimums().y >= b.maximums().y {
        return false;
    }
    if a.maximums().y <= b.minimums().y {
        return false;
    }

    if a.minimums().z >= b.maximums().z {
        return false;
    }
    if a.maximums().z <= b.minimums().z {
        return false;
    }

    // Overlap on all three axes, so their intersection must be non-empty
    true
}
