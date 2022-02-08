// TODO: Most of this is from bevy_mod_bound, want to use this as example to myself,
// taking a look at debug render speed, at 2k objects
// cuts fps by ~70%, going to look at using polyline and removing the child transform and mesh
pub mod aabb;
pub mod debug;
pub mod obb;
pub mod sphere;

use bevy::{prelude::*, transform::TransformSystem};
use debug::{update_debug_mesh_visibility, update_debug_meshes};
use std::fmt::Debug;
use std::marker::PhantomData;
use std::ops::{Add, AddAssign};

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemLabel)]
pub enum BoundingSystem {
    UpdateBounds,
    UpdateDebug,
    UpdateDebugVisibility,
}

#[derive(Default)]
pub struct BoundingVolumePlugin<T: BoundingVolume> {
    marker: std::marker::PhantomData<T>,
}

/// A plugin that provides functionality for generating and updating bounding volumes for meshes.
impl<T> Plugin for BoundingVolumePlugin<T>
where
    T: 'static + Send + Sync + BoundingVolume + Clone + Debug + Component,
    Mesh: From<&'static T>,
{
    fn build(&self, app: &mut App) {
        app.add_system_to_stage(CoreStage::PreUpdate, spawn::<T>)
            .add_system_to_stage(
                CoreStage::PostUpdate,
                update::<T>
                    .after(TransformSystem::TransformPropagate)
                    .label(BoundingSystem::UpdateBounds),
            )
            .add_system_to_stage(
                CoreStage::PostUpdate,
                update_debug_meshes::<T>
                    .after(BoundingSystem::UpdateBounds)
                    .label(BoundingSystem::UpdateDebug),
            )
            .add_system_to_stage(
                CoreStage::PostUpdate,
                update_debug_mesh_visibility::<T>
                    .after(BoundingSystem::UpdateDebug)
                    // TODO: Label here could be wrong
                    .before(bevy::render::view::visibility::VisibilitySystems::CheckVisibility),
            );
    }
}

/// Marks an entity to have a bounding volume generated. This entity should have a [Mesh]
/// component. A bounding volume component of type T will be computed and added to the entity once
/// the aforementioned mesh has loaded and can be read. This ensures that bounding volume
/// components are always valid when queried, and at worst case can only be out of date if queried
/// in a frame before the bounding volume update system is run.
#[derive(Debug, Clone, Component)]
pub struct Bounded<T: BoundingVolume + Send + Sync>(PhantomData<T>);

impl<T: BoundingVolume + Send + Sync> Default for Bounded<T> {
    fn default() -> Self {
        Bounded(PhantomData::default())
    }
}

/// A [BoundingVolume] stores its properties in mesh space to maximize precision. Because some types
/// of bounding volume must be recomputed if the mesh is scaled or rotated, this trait calls an
/// update function depending on whether the mesh or transform has updated.
pub trait BoundingVolume {
    /// Initializes a valid bounding volume given a [Mesh] and [GlobalTransform].
    fn new(mesh: &Mesh, transform: &GlobalTransform) -> Self;
    /// Generate a debug [Mesh] representing the bounding volume from a [BoundingVolume].
    fn new_debug_mesh(&self, transform: &GlobalTransform) -> Mesh;
    /// This function is only called when only the entity's [GlobalTransform] has changed. Only
    /// some types of bounding volume need to be recomputed in this case.
    fn update_on_transform_change(
        &self,
        _mesh: &Mesh,
        _transform: &GlobalTransform,
    ) -> Option<Self>
    where
        Self: Sized;

    /// Returns true if the bounding mesh is entirely on the outside of the supplied plane.
    /// "Outside" is the direction that the plane normal points to.
    fn outside_plane(
        &self,
        bound_vol_position: &GlobalTransform,
        point: Vec3,
        normal: Vec3,
    ) -> bool;
}

/// Spawns a new [BoundingVolume], replacing the [AddBoundingVolume] marker component on the
/// entity. This new BoundingVolume is fully initialized and will be kept up to date with the
/// `update()` system.
#[allow(clippy::type_complexity)]
pub fn spawn<T: 'static + BoundingVolume + Send + Sync + Debug + Component>(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    query: Query<(&Handle<Mesh>, &GlobalTransform, Entity), With<Bounded<T>>>,
) {
    for (handle, transform, entity) in query.iter() {
        if let Some(mesh) = meshes.get(handle) {
            let new_bound = T::new(mesh, transform);
            //info!("New bounding volume generated: {:?}", new_bound);
            commands
                .entity(entity)
                .insert(new_bound)
                .remove::<Bounded<T>>();
        }
    }
}

/// Updates [BoundingVolume]s when their meshes or [GlobalTransform]s are changed. If an entity's
/// mesh has changed, triggering a bounding volume update, the update function will won't update it
/// a second time if the transform has also changed.
fn update<T: 'static + BoundingVolume + Send + Sync + Component>(
    meshes: Res<Assets<Mesh>>,
    changed_mesh_query: Query<Entity, Changed<Handle<Mesh>>>,
    changed_transform_query: Query<Entity, Changed<GlobalTransform>>,
    mut bound_vol_query: Query<(&mut T, &GlobalTransform, &Handle<Mesh>)>,
) {
    for entity in changed_mesh_query.iter() {
        if let Ok((mut bounding_vol, transform, handle)) = bound_vol_query.get_mut(entity) {
            if let Some(mesh) = meshes.get(handle) {
                *bounding_vol = T::new(mesh, transform);
            }
        }
    }
    for entity in changed_transform_query.iter() {
        // Only process entities that haven't already been updated.
        if changed_mesh_query.get(entity).is_err() {
            if let Ok((mut bounding_vol, transform, handle)) = bound_vol_query.get_mut(entity) {
                if let Some(mesh) = meshes.get(handle) {
                    if let Some(bound_vol) =
                        bounding_vol.update_on_transform_change(mesh, transform)
                    {
                        *bounding_vol = bound_vol;
                    }
                }
            }
        }
    }
}

/// A bounding volume for object in local space
#[derive(Component, Copy, Clone, Debug, PartialEq)]
pub struct Bounds {
    pub mins: Vec3,
    pub maxs: Vec3,
}

impl Default for Bounds {
    fn default() -> Self {
        Self {
            mins: Vec3::splat(std::f32::MAX),
            maxs: Vec3::splat(-std::f32::MAX),
        }
    }
}

impl Bounds {
    pub fn from_points(pts: &[Vec3]) -> Self {
        pts.iter().fold(Bounds::default(), |acc, pt| acc + *pt)
    }
    // fn clear(&mut self) {
    //     *self = Self::new();
    // }

    // fn does_intersect(&self, rhs: &Self) -> bool {
    //     if self.maxs.cmplt(rhs.mins).any() {
    //         false
    //     } else if rhs.maxs.cmplt(self.mins).any() {
    //         false
    //     } else {
    //         true
    //     }
    // }

    // fn expand_by_points(&mut self, points: &[Vec3]) {
    //     for point in points {
    //         self.expand_by_point(*point);
    //     }
    // }

    pub fn expand_by_point(&mut self, rhs: Vec3) {
        self.mins = Vec3::select(rhs.cmplt(self.mins), rhs, self.mins);
        self.maxs = Vec3::select(rhs.cmpgt(self.maxs), rhs, self.maxs);
    }

    // fn expand_by_bounds(&mut self, rhs: &Self) {
    //     self.expand_by_point(rhs.mins);
    //     self.expand_by_point(rhs.maxs);
    // }

    pub fn width(&self) -> Vec3 {
        self.maxs - self.mins
    }

    // fn width_x(&self) -> f32 {
    //     self.maxs.x - self.mins.x
    // }

    // fn width_y(&self) -> f32 {
    //     self.maxs.y - self.mins.y
    // }

    // fn width_z(&self) -> f32 {
    //     self.maxs.z - self.mins.z
    // }
}

impl Add<Vec3> for Bounds {
    type Output = Self;
    fn add(self, pt: Vec3) -> Self::Output {
        Bounds {
            mins: Vec3::select(pt.cmplt(self.mins), pt, self.mins),
            maxs: Vec3::select(pt.cmpgt(self.maxs), pt, self.maxs),
        }
    }
}

impl AddAssign<Vec3> for Bounds {
    fn add_assign(&mut self, pt: Vec3) {
        self.mins = Vec3::select(pt.cmplt(self.mins), pt, self.mins);
        self.maxs = Vec3::select(pt.cmpgt(self.maxs), pt, self.maxs);
    }
}
