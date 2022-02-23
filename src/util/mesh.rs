// MESH - Implements mesh loading, bvh construction (todo), and bvh intersection (todo)

use std::borrow::BorrowMut;
use tobj::{self, Mesh};
use cgmath::*;
use std::mem;
use rand::Rng;

use super::tracing::{self, Intersectable, Triangle};

type Vec3 = Vector3<f32>;

#[derive(Debug, Clone, Copy)]
pub struct AABB {
    pub min: Vec3,
    pub max: Vec3,
}
impl AABB {
    // returns the bounding box surrounding two given bounding boxes
    fn aabb_surrounding(a: &AABB, b: &AABB) -> AABB {
        AABB {
            min: vec3(
                f32::min(a.min.x, b.min.x),
                f32::min(a.min.y, b.min.y),
                f32::min(a.min.z, b.min.z),
            ),
            max: vec3(
                f32::max(a.max.x, b.max.x),
                f32::max(a.max.y, b.max.y),
                f32::max(a.max.z, b.max.z),
            ),
        }
    }
}
impl Default for AABB {
    fn default() -> AABB {
        AABB {
           min: Vec3::zero(), max: Vec3::zero(),
        }
    }
}
impl tracing::Intersectable for AABB {
    // this doesn't actually use the RayHit struct, so for now it just returns Some default or None
    fn intersect_ray(&self, ray: &tracing::Ray, t_min: f32, t_max: f32) -> Option<tracing::RayHit> {
        // based on raytracing the next week
        let mut tmin = t_min.clone();
        let mut tmax = t_max.clone();
        for axis in 0..3 {
            let inv_d = 1.0 / ray.direction[axis];
            let mut t0 = (self.min[axis] - ray.origin[axis]) * inv_d;
            let mut t1 = (self.max[axis] - ray.origin[axis]) * inv_d;
            if inv_d < 0.0 {
                mem::swap(&mut t0, &mut t1);
            }
            tmin = f32::max(t0, tmin);
            tmax = f32::min(t1, tmax);
            if tmax <= tmin {
                return None;
            }
        }
        return Some(tracing::RayHit {
            distance: 0.0,
            hitpoint: Vec3::zero(),
            normal: Vec3::zero(),
            albedo: Vec3::zero(),
        })
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(self.clone())
    }
}

#[derive(Default)]
pub struct BVHNode {
    pub aabb: AABB,
    pub left: Option<Box<BVHNode>>,
    pub right: Option<Box<BVHNode>>,
    pub primitive: Option<Box<dyn Intersectable>>,
}
impl BVHNode {
    pub fn build_from_mesh(mesh: &Mesh) -> Box<BVHNode> {
        print!("Building BVH...");
        let mut objects = Vec::new();
        // populate array of total objects (inefficient for now)
        // aka: iterate over triangles
        for i in (0..mesh.indices.len()).step_by(3) {
            // load vertices
            let (x,y,z) = (mesh.indices[i] as usize, mesh.indices[i+1] as usize, mesh.indices[i+2] as usize);
            let a = vec3(mesh.positions[x*3], mesh.positions[x*3+1], mesh.positions[x*3+2]);
            let b = vec3(mesh.positions[y*3], mesh.positions[y*3+1], mesh.positions[y*3+2]);
            let c = vec3(mesh.positions[z*3], mesh.positions[z*3+1], mesh.positions[z*3+2]);
           // create triangle
           objects.push(tracing::Triangle {a: a, b: b, c: c, albedo: vec3(0.5,0.5,0.5)});
        }
        let start: usize = 0;
        let end = objects.len();
        let node = BVHNode::build_from_mesh_helper(&mut objects, start, end);        
        println!("Done.");
        return node;
    }
    fn build_from_mesh_helper(objects: &mut Vec<tracing::Triangle>, start: usize, end: usize) -> Box<BVHNode> {
        let mut node = BVHNode::default();
        if end-start == 1 {
            // make the node a leaf
            node.primitive = Some(Box::new(objects[start].clone()));
            node.aabb = objects[start].bounding_box().unwrap_or_default();
        }
        else {
            // sort segment by random axis
            let mut rng = rand::thread_rng();
            let axis: usize = rng.gen_range(0..3);
            let comparator = |a: &Triangle, b: &Triangle| {
                let f = a.bounding_box().unwrap_or_default().min[axis];
                let g = b.bounding_box().unwrap_or_default().min[axis];
                f.partial_cmp(&g).unwrap_or(std::cmp::Ordering::Equal)
            };
            objects[start..end].sort_by(comparator);
            // recurse on each side
            let mid = start + (end-start)/2;
            let left  = Self::build_from_mesh_helper(objects, start, mid);
            let right = Self::build_from_mesh_helper(objects, mid, end);
            node.aabb = AABB::aabb_surrounding(&left.aabb, &right.aabb);
            node.left = Some(left);
            node.right = Some(right);
        }
        Box::new(node)
    }

}
impl tracing::Intersectable for BVHNode {
    fn intersect_ray(&self, ray: &tracing::Ray, t_min: f32, t_max: f32) -> Option<tracing::RayHit> {
        if let Some(prim) = &self.primitive {
            // node is a leaf
            prim.intersect_ray(ray, t_min, t_max)
        }
        else {
            // node is interior - check if ray intersects aabb
            if self.aabb.intersect_ray(ray, t_min, t_max).is_some() {
                // recurse to children
                if let Some(left_node) = &self.left {
                    let hit = left_node.intersect_ray(ray, t_min, t_max);
                    if hit.is_some() { return hit }
                }
                if let Some(right_node) = &self.right {
                    let hit = right_node.intersect_ray(ray, t_min, t_max);
                    if hit.is_some() { return hit }
                }
            }
            // ray misses this node entirely
            None
            
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(self.aabb.clone())
    }
}


fn compute_normals(mesh: &mut Mesh) {
    assert!(mesh.normals.is_empty());
    mesh.normals = vec![0.0; mesh.positions.len()];
    // iterate over triangles
    for i in (0..mesh.indices.len()).step_by(3) {
        // load vertices
        let (x,y,z) = (mesh.indices[i] as usize, mesh.indices[i+1] as usize, mesh.indices[i+2] as usize);
        let a = vec3(mesh.positions[x*3], mesh.positions[x*3+1], mesh.positions[x*3+2]);
        let b = vec3(mesh.positions[y*3], mesh.positions[y*3+1], mesh.positions[y*3+2]);
        let c = vec3(mesh.positions[z*3], mesh.positions[z*3+1], mesh.positions[z*3+2]);
        // compute normal
        let n = 0.5* (b-a).cross(c-a);
        // accumulate into normals array
        mesh.normals[x*3] += n.x; mesh.normals[x*3+1] += n.y; mesh.normals[x*3+2] += n.z;
        mesh.normals[y*3] += n.x; mesh.normals[y*3+1] += n.y; mesh.normals[y*3+2] += n.z;
        mesh.normals[z*3] += n.x; mesh.normals[z*3+1] += n.y; mesh.normals[z*3+2] += n.z;
    }
    // normalize all normals
    for i in (0..mesh.normals.len()).step_by(3) {
        let m = vec3(mesh.normals[i],mesh.normals[i+1],mesh.normals[i+2]).magnitude();
        mesh.normals[i] /= m;
        mesh.normals[i+1] /= m;
        mesh.normals[i+2] /= m;
    }
    println!("Computed {} normals", mesh.normals.len() / 3);
}


pub fn get_mesh() -> Mesh {
    // load obj
    let obj = tobj::load_obj(
        "./obj/teapot.obj",
        &tobj::LoadOptions {
            single_index: true,
            triangulate: true,
            ignore_points: false,
            ignore_lines: false,
        },
    );
    assert!(obj.is_ok());
    let (mut models, materials) = obj.expect("Failed to load OBJ file");
    let materials = materials.expect("Failed to load MTL file");
    println!("Loaded {} successfully:", "./obj/teapot.obj");
    println!("# of models: {}", models.len());
    println!("# of materials: {}", materials.len());
    
    let teapot = (models[0].mesh).borrow_mut();
    // // compute normals if not present
    // if teapot.normals.is_empty() {
    //     compute_normals(teapot);
    // }
    // assert!(!teapot.normals.is_empty());
    teapot.clone()
}