use crate::bounds::*;
use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

// TODO: Not really happy with putting this much data into an enum
// since will all be the same size in memory, did turn this into a
// trait, but that caused more problems, this needs more testing

#[derive(Inspectable, Copy, Clone, Debug)]
pub enum PyhsicsShape {
    Sphere {
        radius: f32,
    },
    Box {
        points: [Vec3; 8],
        bounds: Bounds,
        com: Vec3,
    },
}

impl Default for PyhsicsShape {
    fn default() -> PyhsicsShape {
        PyhsicsShape::Sphere { radius: 1.0 }
    }
}

impl PyhsicsShape {

    pub fn new_box(points: &[Vec3]) -> Self {
        assert!(!points.is_empty());

        let mut bounds = Bounds::default();
        for p in points {
            bounds.expand_by_point(*p);
        }

        let points = [
            Vec3::new(bounds.mins.x, bounds.mins.y, bounds.mins.z),
            Vec3::new(bounds.maxs.x, bounds.mins.y, bounds.mins.z),
            Vec3::new(bounds.mins.x, bounds.maxs.y, bounds.mins.z),
            Vec3::new(bounds.mins.x, bounds.mins.y, bounds.maxs.z),
            Vec3::new(bounds.maxs.x, bounds.maxs.y, bounds.maxs.z),
            Vec3::new(bounds.mins.x, bounds.maxs.y, bounds.maxs.z),
            Vec3::new(bounds.maxs.x, bounds.mins.y, bounds.maxs.z),
            Vec3::new(bounds.maxs.x, bounds.maxs.y, bounds.mins.z),
        ];

        let com = (bounds.maxs + bounds.mins) * 0.5;

        PyhsicsShape::Box {
            points,
            bounds,
            com,
        }
    }

    pub fn centre_of_mass(&self) -> Vec3 {
        match self {
            PyhsicsShape::Sphere { .. } => Vec3::ZERO,
            PyhsicsShape::Box {
                points: _points,
                bounds: _bounds,
                com,
            } => *com,
        }
    }

    pub fn inertia_tensor(&self) -> Mat3 {
        match self {
            PyhsicsShape::Sphere { radius } => {
                Mat3::from_diagonal(Vec3::splat(2.0 * radius * radius / 5.0))
            }
            PyhsicsShape::Box {
                points: _points,
                bounds,
                com: _com,
            } => {
                // inertia tensor for box centered around zero
                let d = bounds.maxs - bounds.mins;
                let dd = d * d;
                let diagonal = Vec3::new(dd.y + dd.z, dd.x + dd.z, dd.x + dd.y) / 12.0;
                let tensor = Mat3::from_diagonal(diagonal);

                // now we need to use the parallel axis theorem to get the ineria tensor for a box that is
                // not centered around the origin

                let cm = (bounds.maxs + bounds.mins) * 0.5;

                // the displacement from the center of mass to the origin
                let r = -cm;
                let r2 = r.length_squared();

                let pat_tensor = Mat3::from_cols(
                    Vec3::new(r2 - r.x * r.x, r.x * r.y, r.x * r.z),
                    Vec3::new(r.y * r.x, r2 - r.y * r.y, r.y * r.z),
                    Vec3::new(r.z * r.x, r.z * r.y, r2 - r.z * r.z),
                );

                // now we need to add the centre of mass tensor and the parallel axis theorem tensor
                // together
                tensor + pat_tensor
            }
        }
    }

    pub fn bounds(&self, transform: &Transform) -> Bounds {
        match self {
            PyhsicsShape::Sphere { radius } => Bounds {
                mins: Vec3::splat(-radius) + transform.translation,
                maxs: Vec3::splat(*radius) + transform.translation,
            },
            PyhsicsShape::Box {
                points,
                bounds,
                com,
            } => {
                let corners = [
                    Vec3::new(bounds.mins.x, bounds.mins.y, bounds.mins.z),
                    Vec3::new(bounds.mins.x, bounds.mins.y, bounds.maxs.z),
                    Vec3::new(bounds.mins.x, bounds.maxs.y, bounds.mins.z),
                    Vec3::new(bounds.maxs.x, bounds.mins.y, bounds.mins.z),
                    Vec3::new(bounds.maxs.x, bounds.maxs.y, bounds.maxs.z),
                    Vec3::new(bounds.maxs.x, bounds.maxs.y, bounds.mins.z),
                    Vec3::new(bounds.maxs.x, bounds.mins.y, bounds.maxs.z),
                    Vec3::new(bounds.mins.x, bounds.maxs.y, bounds.maxs.z),
                ];

                let mut bounds = Bounds::default();
                for pt in &corners {
                    let pt = (transform.rotation * *pt) + transform.translation;
                    bounds.expand_by_point(pt);
                }

                bounds
            }
        }
    }

    fn support(&self, dir: Vec3, transform: &Transform, bias: f32) -> Vec3 {
        match self {
            PyhsicsShape::Sphere { radius } => transform.translation + dir * (radius + bias),
            PyhsicsShape::Box {
                points,
                bounds,
                com,
            } => {
                // find the point in the furthest in direction
                let mut max_pt = (transform.rotation * points[0]) + transform.translation;
                let mut max_dist = dir.dot(max_pt);
                for pt in &points[1..] {
                    let pt = (transform.rotation * *pt) + transform.translation;
                    let dist = dir.dot(pt);
                    if dist > max_dist {
                        max_dist = dist;
                        max_pt = pt;
                    }
                }

                let norm = dir.normalize() * bias;

                max_pt + norm
            }
        }
    }

    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        match self {
            PyhsicsShape::Sphere { radius } => unimplemented!(),
            PyhsicsShape::Box {
                points,
                bounds,
                com,
            } => {
                let mut max_speed = 0.0;
                for pt in points.iter() {
                    let r = *pt - *com;
                    let linear_velocity = angular_velocity.cross(r);
                    let speed = dir.dot(linear_velocity);
                    if speed > max_speed {
                        max_speed = speed;
                    }
                }
                max_speed
            }
        }
    }
}
