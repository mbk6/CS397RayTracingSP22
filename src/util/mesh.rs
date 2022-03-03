// MESH - Implements mesh loading, bvh construction (todo), and bvh intersection (todo)

use std::{borrow::{BorrowMut, Borrow}, sync::Arc};
use tobj::{self, Mesh};
use cgmath::*;
use std::mem;
use rand::Rng;

use super::tracing::{self, Intersectable, Triangle, Ray, RayHit};

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
impl Intersectable for AABB {
    // this doesn't actually use the RayHit struct, so for now it just returns Some default or None
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
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
        return Some(RayHit {
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


#[derive(Debug, Clone, Default)]
pub struct BVHNode {
    pub aabb: AABB,
    pub left: Option<Box<BVHNode>>,
    pub right: Option<Box<BVHNode>>,
    // pub primitive: Option<Box<dyn Intersectable>>,
    pub primitive: Option<IndexedTriangle>,
}
impl BVHNode {
    // pub fn build_from_mesh(sm: &StaticMesh) -> Box<BVHNode> {
    //     print!("Building BVH...");
    //     let mut objects = Vec::new();
    //     // populate array of total objects (inefficient for now)
    //     // aka: iterate over triangles
    //     for i in 0..sm.mesh.indices.len()/3 {
    //         // load vertices
    //         let (a,b,c) = sm.get_triangle(i);
    //        // create triangle
    //        objects.push(Triangle {a: a, b: b, c: c, albedo: vec3(0.5,0.5,0.5)});
    //     }
    //     let start: usize = 0;
    //     let end = objects.len();
    //     let node = BVHNode::build_from_mesh_helper(&mut objects, start, end);        
    //     println!("Done.");
    //     return node;
    // }
    // fn build_from_mesh_helper(objects: &mut Vec<Triangle>, start: usize, end: usize) -> Box<BVHNode> {
    //     let mut node = BVHNode::default();
    //     if end-start == 1 {
    //         // make the node a leaf
    //         node.primitive = Some(Box::new(objects[start].clone()));
    //         node.aabb = objects[start].bounding_box().unwrap_or_default();
    //     }
    //     else {
    //         // sort segment by random axis
    //         let mut rng = rand::thread_rng();
    //         let axis: usize = rng.gen_range(0..3);
    //         let comparator = |a: &Triangle, b: &Triangle| {
    //             let f = a.bounding_box().unwrap_or_default().min[axis];
    //             let g = b.bounding_box().unwrap_or_default().min[axis];
    //             f.partial_cmp(&g).unwrap_or(std::cmp::Ordering::Equal)
    //         };
    //         objects[start..end].sort_by(comparator);
    //         // recurse on each side
    //         let mid = start + (end-start)/2;
    //         let left  = Self::build_from_mesh_helper(objects, start, mid);
    //         let right = Self::build_from_mesh_helper(objects, mid, end);
    //         node.aabb = AABB::aabb_surrounding(&left.aabb, &right.aabb);
    //         node.left = Some(left);
    //         node.right = Some(right);
    //     }
    //     Box::new(node)
    // }

}
impl Intersectable for BVHNode {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        if let Some(prim) = &self.primitive {
            // node is a leaf
            prim.intersect_ray(ray, t_min, t_max)
        }
        else {
            // node is interior - check if ray intersects aabb
            let mut best_hit = None;
            let mut best_t = t_max.clone();
            if self.aabb.intersect_ray(ray, t_min, t_max).is_some() {
                // recurse to children
                if let Some(left_node) = &self.left {
                    let hit_opt = left_node.intersect_ray(ray, t_min, t_max);
                    if let Some(hit) = hit_opt { 
                        best_hit = hit_opt;
                        best_t = hit.distance;
                    }
                }
                if let Some(right_node) = &self.right {
                    let hit_opt = right_node.intersect_ray(ray, t_min, best_t);
                    if hit_opt.is_some() { best_hit = hit_opt; }
                }
            }
            // ray misses this node entirely
            best_hit
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(self.aabb.clone())
    }
}

#[derive(Debug, Clone)]
pub struct StaticMesh {
    mesh: Arc<Mesh>,
    material: Vec3, // materials to be implemented soon - right now just albedo
    transform: Matrix4<f32>, // transforms to be implemented soon
    bvh_root: Option<Box<BVHNode>>,
}
impl StaticMesh {
    // pub fn compute_normals(&mut self) {
    //     if !self.mesh.normals.is_empty() { return };
    //     self.mesh.normals = vec![0.0; self.mesh.positions.len()];
    //     // iterate over triangles
    //     for i in 0..self.mesh.indices.len()/3 {
    //         // load vertices
    //         let (a,b,c) = self.get_triangle(i);
    //         // compute normal
    //         let n = 0.5* (b-a).cross(c-a);
    //         // accumulate into normals array
    //         let (x,y,z) = (self.mesh.indices[i*3] as usize, self.mesh.indices[i*3+1] as usize, self.mesh.indices[i*3+2] as usize);
    //         self.mesh.normals[x*3] += n.x; self.mesh.normals[x*3+1] += n.y; self.mesh.normals[x*3+2] += n.z;
    //         self.mesh.normals[y*3] += n.x; self.mesh.normals[y*3+1] += n.y; self.mesh.normals[y*3+2] += n.z;
    //         self.mesh.normals[z*3] += n.x; self.mesh.normals[z*3+1] += n.y; self.mesh.normals[z*3+2] += n.z;
    //     }
    //     // normalize all normals
    //     for i in 0..self.mesh.normals.len()/3 {
    //         let m = vec3(self.mesh.normals[i*3],self.mesh.normals[i*3+1],self.mesh.normals[i*3+2]).magnitude();
    //         self.mesh.normals[i*3] /= m;
    //         self.mesh.normals[i*3+1] /= m;
    //         self.mesh.normals[i*3+2] /= m;
    //     }
    //     println!("Computed {} normals", self.mesh.normals.len() / 3);
    // }
    
    pub fn load_from_file(file_name: &str) -> StaticMesh {
        // load obj
        let obj = tobj::load_obj(
            file_name,
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
        println!("Loaded {} successfully:", file_name);
        println!("# of models: {}", models.len());
        println!("# of materials: {}", materials.len());
        
        // assume for now that there's only one mesh
        let mut sm = StaticMesh { 
            mesh: Arc::new(models.remove(0).mesh),
            bvh_root: None,
            material: vec3(0.0,0.5,0.2),
            transform: Matrix4::zero(),
        };
        //sm.compute_normals();
        sm.build_bvh();
        sm
    }

    pub fn build_bvh(&mut self) {
        if self.bvh_root.is_some() { return }
        print!("Building BVH...");
        // make temporary array of total triangles
        let mut tris = Vec::new();
        for i in 0..self.mesh.indices.len()/3 {
            tris.push(IndexedTriangle { idx: i, sm: self.mesh.clone() })
        }
        let start: usize = 0;
        let end = tris.len();
        let node = self.build_bvh_helper(&mut tris, start, end);        
        self.bvh_root = Some(node);
        println!("Done.");
    }
    fn build_bvh_helper(&self, tris: &mut Vec<IndexedTriangle>, start: usize, end: usize) -> Box<BVHNode> { // start/end = triangle indices in range (0..indices.len()/3)
        let mut node = BVHNode::default();
        if end-start == 1 {
            // make the node a leaf
            let tri = IndexedTriangle { idx: start, sm: self.mesh.clone()};
            node.aabb = tri.bounding_box().unwrap_or_default();
            node.primitive = Some(tri);
        }
        else {
            // sort segment by random axis
            let mut rng = rand::thread_rng();
            let axis: usize = rng.gen_range(0..3);
            let comparator = |a: &IndexedTriangle, b: &IndexedTriangle| {
                let f = a.bounding_box().unwrap_or_default().min[axis];
                let g = b.bounding_box().unwrap_or_default().min[axis];
                f.partial_cmp(&g).unwrap_or(std::cmp::Ordering::Equal)
            };
            tris[start..end].sort_by(comparator);
            // recurse on each side
            let mid = start + (end-start)/2;
            let left  = self.build_bvh_helper(tris, start, mid);
            let right = self.build_bvh_helper(tris, mid, end);
            node.aabb = AABB::aabb_surrounding(&left.aabb, &right.aabb);
            node.left = Some(left);
            node.right = Some(right);
        }
        Box::new(node)
    }

    // retrieves the idx'th triangle from the mesh
    pub fn get_triangle(&self, idx: usize) -> (Vec3, Vec3, Vec3) {
        let (x,y,z) = (self.mesh.indices[idx*3] as usize, self.mesh.indices[idx*3+1] as usize, self.mesh.indices[idx*3+2] as usize);
        let a = vec3(self.mesh.positions[x*3], self.mesh.positions[x*3+1], self.mesh.positions[x*3+2]);
        let b = vec3(self.mesh.positions[y*3], self.mesh.positions[y*3+1], self.mesh.positions[y*3+2]);
        let c = vec3(self.mesh.positions[z*3], self.mesh.positions[z*3+1], self.mesh.positions[z*3+2]);
        (a,b,c)
    }
    pub fn get_triangle_from_mesh(mesh: &Mesh, idx: usize) -> (Vec3, Vec3, Vec3) {
        let (x,y,z) = (mesh.indices[idx*3] as usize, mesh.indices[idx*3+1] as usize, mesh.indices[idx*3+2] as usize);
        let a = vec3(mesh.positions[x*3], mesh.positions[x*3+1], mesh.positions[x*3+2]);
        let b = vec3(mesh.positions[y*3], mesh.positions[y*3+1], mesh.positions[y*3+2]);
        let c = vec3(mesh.positions[z*3], mesh.positions[z*3+1], mesh.positions[z*3+2]);
        (a,b,c)
    }
}
impl Intersectable for StaticMesh {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        match &self.bvh_root {
            Some(node) => node.intersect_ray(ray, t_min, t_max),
            None => None
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        match &self.bvh_root {
            Some(node) => node.bounding_box(),
            None => None
        }
    }
}

#[derive(Debug, Clone)]
pub struct IndexedTriangle {
    // represents a triangle in an indexed-triangle data structure
    pub idx: usize,
    pub sm: Arc<Mesh>,
}
impl Intersectable for IndexedTriangle {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // lookup vertex data from mesh
        let (a,b,c) = StaticMesh::get_triangle_from_mesh(&self.sm, self.idx);
        // usual ray-triangle intersection
        const EPSILON : f32 = 0.0001;
        let e1 = b - a;
        let e2 = c - a;
        let q = ray.direction.cross(e2);
        let g = e1.dot(q);
        if g.abs() < EPSILON { return None; }
        let f = 1.0/g;
        let s = ray.origin - a;
        let u = f*s.dot(q);
        if u < 0.0 { return None; }
        let r = s.cross(e1);
        let v = f*ray.direction.dot(r);
        if v < 0.0 || u+v > 1.0 { return None }
        let t = f*e2.dot(r);
        let hitpoint = ray.origin + t*ray.direction;
        if t < t_min || t > t_max { return None }
        return Some(RayHit {
            distance: t,
            hitpoint: hitpoint,
            normal: e1.cross(e2).normalize(),
            albedo: vec3(0.3, 0.0, 0.5),
        })
    }
    fn bounding_box(&self) -> Option<AABB> {
        let (a,b,c) = StaticMesh::get_triangle_from_mesh(&self.sm, self.idx);
        Some(AABB {
            min: vec3(
                f32::min(a.x,f32::min(b.x, c.x)),
                f32::min(a.y,f32::min(b.y, c.y)),
                f32::min(a.z,f32::min(b.z, c.z))
            ),
            max: vec3(
                f32::max(a.x,f32::max(b.x, c.x)),
                f32::max(a.y,f32::max(b.y, c.y)),
                f32::max(a.z,f32::max(b.z, c.z))
            ),
        })
    }
}