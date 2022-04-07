// GEOMETRY - Implements geometric primitives, mesh loading, bvh construction/intersection

use std::{sync::Arc};
use tobj::{self, Mesh};
use cgmath::*;
use std::mem;
use rand::Rng;

use super::tracing::*;
use super::materials::*;


////////////////////////////////////////////////////////
/////   INTERSECTABLES
////////////////////////////////////////////////////////

// AXIS-ALIGNED BOUNDING BOX
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
            frontface: true,
            distance: 0.0,
            hitpoint: Vec3::zero(),
            normal: Vec3::zero(),
            material: Arc::new(Lambertian::default()),
        })
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(self.clone())
    }
}

// BOUNDING VOLUME HIERARCHY - tree of bounding boxes and primitives
#[derive(Debug, Clone, Default)]
pub struct BVHNode {
    pub aabb: AABB,
    pub left: Option<Box<BVHNode>>,
    pub right: Option<Box<BVHNode>>,
    // pub primitive: Option<Box<dyn Intersectable>>,
    pub primitive: Option<IndexedTriangle>,
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
                        best_hit = Some(hit.clone());
                        best_t = hit.distance;
                    }
                }
                if let Some(right_node) = &self.right {
                    let hit_opt = right_node.intersect_ray(ray, t_min, best_t);
                    if hit_opt.is_some() { best_hit = hit_opt.clone(); }
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

// STATIC MESH
#[derive(Clone)]
pub struct StaticMesh {
    mesh: Arc<Mesh>,
    material: Arc<dyn Material + Send + Sync>,
    bvh_root: Option<Box<BVHNode>>,
    transform: Matrix4<f32>,
    inv_transform: Matrix4<f32>,
}
impl StaticMesh {
    
    // load a mesh from file to create a new StaticMesh object
    pub fn load_from_file(file_name: &str, material: Arc<dyn Material + Sync + Send>, transform: Matrix4<f32>) -> StaticMesh {
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
            material: material.clone(),
            transform: transform,
            inv_transform: transform.inverse_transform().unwrap(),
        };
        //sm.compute_normals();
        sm.build_bvh();
        sm
    }

    // build the StaticMesh's bvh using its mesh
    pub fn build_bvh(&mut self) {
        if self.bvh_root.is_some() { return }
        print!("Building BVH...");
        // make temporary array of total triangles
        let mut tris = Vec::new();
        for i in 0..self.mesh.indices.len()/3 {
            tris.push(IndexedTriangle { idx: i, mesh: self.mesh.clone() })
        }
        let start: usize = 0;
        let end = tris.len();
        let node = self.build_bvh_helper(&mut tris, start, end);        
        self.bvh_root = Some(node);
        println!("Done.");
    }
    // helper for bvh construction recursion
    fn build_bvh_helper(&self, tris: &mut Vec<IndexedTriangle>, start: usize, end: usize) -> Box<BVHNode> { // start/end = triangle indices in range (0..indices.len()/3)
        let mut node = BVHNode::default();
        if end-start == 1 {
            // make the node a leaf
            let tri = IndexedTriangle { idx: start, mesh: self.mesh.clone()};
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
        Self::get_triangle_from_mesh(&self.mesh, idx)
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
        // intersect bvh but replace material data
        if let Some(root) = &self.bvh_root {
            let transformed_ray = Ray { origin: self.inv_transform.transform_point(point3(ray.origin.x, ray.origin.y, ray.origin.z)).to_vec(), direction: self.inv_transform.transform_vector(ray.direction) };
            if let Some(mut hit) = root.intersect_ray(&transformed_ray, t_min, t_max) {
                hit.material = self.material.clone();
                return Some(hit);
            }
        }
        return None;
    }
    fn bounding_box(&self) -> Option<AABB> {
        match &self.bvh_root {
            Some(node) => node.bounding_box(),
            None => None
        }
    }
}

// INDEXED TRIANGLE - triangle object that references data in an indexed-mesh structure
#[derive(Debug, Clone)]
pub struct IndexedTriangle {
    // represents a triangle in an indexed-triangle data structure
    pub idx: usize,
    pub mesh: Arc<Mesh>,
}
impl Intersectable for IndexedTriangle {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // lookup vertex data from mesh
        let (a,b,c) = StaticMesh::get_triangle_from_mesh(&self.mesh, self.idx);
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
        
        Some(RayHit::new(t, e1.cross(e2).normalize(), Arc::new(Lambertian::default()), ray))
    }
    fn bounding_box(&self) -> Option<AABB> {
        let (a,b,c) = StaticMesh::get_triangle_from_mesh(&self.mesh, self.idx);
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


////////////////////////////////////////////////////////
/////   PRIMITIVES
////////////////////////////////////////////////////////
// SPHERE
pub struct Sphere {
    pub center: Vec3,
    pub radius: f32,
    pub material: Arc<dyn Material + Send + Sync>,
}
impl Intersectable for Sphere {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // ray-sphere intersection
        let f = ray.origin - self.center;
        let a = ray.direction.magnitude2();
        let b = 2.0*f.dot(ray.direction);
        let c = f.magnitude2() - self.radius*self.radius;
        let d = b*b - 4.0*a*c;
        if d < 0.0 {
            None
        }
        else {
            let t1 = (-b - d.sqrt()) / (2.0*a);
            let t2 = (-b + d.sqrt()) / (2.0*a);
            let t = if t1 >= t_min { t1 } else { t2 };
            let hitpoint = ray.origin + t*ray.direction;
            if t < t_min || t > t_max { return None }
            Some(RayHit::new(t, (hitpoint - self.center).normalize(), self.material.clone(), ray))
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(AABB {
            min: self.center - vec3(self.radius,self.radius,self.radius),
            max: self.center + vec3(self.radius,self.radius,self.radius),
        })
    }
}

// TRIANGLE
#[derive(Clone)]
pub struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub material: Arc<dyn Material + Send + Sync>,
}
impl Intersectable for Triangle {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // ray-triangle intersection
        const EPSILON : f32 = 0.0001;
        let e1 = self.b - self.a;
        let e2 = self.c - self.a;
        let q = ray.direction.cross(e2);
        let a = e1.dot(q);
        if a.abs() < EPSILON { return None; }
        let f = 1.0/a;
        let s = ray.origin - self.a;
        let u = f*s.dot(q);
        if u < 0.0 { return None; }
        let r = s.cross(e1);
        let v = f*ray.direction.dot(r);
        if v < 0.0 || u+v > 1.0 { return None }
        let t = f*e2.dot(r);
        let hitpoint = ray.origin + t*ray.direction;
        if t < t_min || t > t_max { return None }

        Some(RayHit::new(t, e1.cross(e2).normalize(), self.material.clone(), ray))
    }
    fn bounding_box(&self) -> Option<AABB> {
        Some(AABB {
            min: vec3(
                f32::min(self.a.x,f32::min(self.b.x, self.c.x)),
                f32::min(self.a.y,f32::min(self.b.y, self.c.y)),
                f32::min(self.a.z,f32::min(self.b.z, self.c.z))
            ),
            max: vec3(
                f32::max(self.a.x,f32::max(self.b.x, self.c.x)),
                f32::max(self.a.y,f32::max(self.b.y, self.c.y)),
                f32::max(self.a.z,f32::max(self.b.z, self.c.z))
            ),
        })
    }
}

// PLANE
pub struct Plane {
    pub point: Vec3,
    pub normal: Vec3,
    pub material: Arc<dyn Material + Send + Sync>,
}
impl Intersectable for Plane {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // ray-plane intersection
        let to_ray_origin = ray.origin - self.point;
        let origin_dist = dot(to_ray_origin, self.normal);
        let n = origin_dist.signum() * self.normal;
        let d = ray.direction.dot(n);
        if d >= 0.0 { 
            None
        }
        else {
            let t = origin_dist.abs() / d.abs();
            if t < t_min || t > t_max { return None }
            let hitpoint = ray.origin + t*ray.direction;

            Some(RayHit::new(t, n, self.material.clone(), ray))
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        None
    }
}

pub struct ConvexVolume {
    pub boundary: Arc<dyn Intersectable + Send + Sync>,
    pub phase_function: Arc<dyn Material + Send + Sync>, 
    pub density: f32,
    // phase function = probabiltiy distrubution function for scattering at each angle (https://www.pbr-book.org/3ed-2018/Volume_Scattering/Phase_Functions)
}
impl Intersectable for ConvexVolume {
    fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<RayHit> {
        // find where ray enters and exits the volume (if at all)
        let hit_entr = self.boundary.intersect_ray(ray, f32::MIN, f32::MAX);
        if hit_entr.is_none() { return None; }
        let t_entr = hit_entr.unwrap().distance;
        let hit_exit = self.boundary.intersect_ray(ray, t_entr+0.0001, f32::MAX);
        if hit_exit.is_none() { return None; }
        let t_exit = hit_exit.unwrap().distance;
        // if ray exits before t_min or enters after t_max, return
        if t_exit < t_min || t_entr > t_max { return None }
        let t_start = f32::max(t_entr, t_min);
        let t_end = f32::min(t_exit, t_max);
        let dist_in_volume = t_end-t_start;
        let dist_before_scatter = (-1.0/self.density) * f32::ln(rand::thread_rng().gen_range(0.0..1.0)); // not sure where this log comes from
        if dist_before_scatter < dist_in_volume {
            // ray scatters t_start + dist_before_scatter forward from its current location
            Some(RayHit::new(t_start+dist_before_scatter, Vec3::zero(), self.phase_function.clone(), ray))
            // (normal and front_face are arbitrary)
        }
        else {
            None // ray passes through without scattering
        }
    }
    fn bounding_box(&self) -> Option<AABB> {
        self.boundary.bounding_box()
    }
}