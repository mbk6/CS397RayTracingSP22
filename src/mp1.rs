// MP1 - based on CS 419 mp1


// includes
use std::{ops::Neg, iter::Scan};
use image::*;
use cgmath::*;
use rand::Rng;

// constants & typedefs
type Vec3 = Vector3<f32>;
type Color = Vec3;

// TRAITS
trait Intersectable {
    fn intersect_ray(&self, ray: &Ray) -> Option<RayHit>;
}

// STRUCTS & ENUMS
struct Camera {
    // camera model based on 419 lectures
    pub eyepoint: Vec3,
    pub view_dir: Vec3,
    pub up: Vec3,
    pub view_plane_dist: f32,
    pub pixel_size: f32,
    pub projection_mode: CameraProjectionMode,
    pub screen_width: u32,
    pub screen_height: u32,
    pub aa_sample_count: u32, // Must be a perfect square
}
pub enum CameraProjectionMode {
    Orthographic,
    Perspective,
}
struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}
struct RayHit {
    pub distance: f32,
    pub hitpoint: Vec3,
    pub normal: Vec3,
    pub albedo: Vec3,
}
struct Scene {
    pub camera: Camera,
    pub objects: Vec<Box<dyn Intersectable>>,
    pub point_light_pos: Vec3,
    pub ambient: Vec3,
}
struct Sphere {
    pub center: Vec3,
    pub radius: f32,
    pub albedo: Vec3,
}
struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub albedo: Vec3,
}
struct Plane {
    pub point: Vec3,
    pub normal: Vec3,
    pub albedo: Vec3,
}

// IMPL
impl Camera {
    // generate camera rays given pixel coordinates and sample count
    // currently uses multi-jittered sampling
    pub fn generate_rays(&self, screen_x: u32, screen_y: u32) -> Vec<Ray> {
        let mut rays = Vec::new();
        let n = self.aa_sample_count as f32;
        let rootn = n.sqrt();
        let mut rng = rand::thread_rng();
        for i in 0..self.aa_sample_count {
            // compute multi-jittered pixel offset
            let rand_x = rng.gen_range(0..self.aa_sample_count) as f32;
            let rand_y = rng.gen_range(0..self.aa_sample_count) as f32;
            let subpixel_x = (i / rootn as u32) as f32;
            let subpixel_y = (i % rootn as u32) as f32;
            let subpixel_offset = vec2(
                (subpixel_x - 0.5*rootn)*self.pixel_size/rootn + (rand_x - 0.5*n)*self.pixel_size/n,
                (subpixel_y - 0.5*rootn)*self.pixel_size/rootn + (rand_y - 0.5*n)*self.pixel_size/n,
             );
            
            // compute pixel center and offset by jitter
            let cam_space_pixel_center = vec3(
                self.pixel_size*(screen_x as f32 - 0.5*(self.screen_width as f32) + 0.5) + subpixel_offset.x,
                self.pixel_size*(0.5 + 0.5*(self.screen_height as f32) - screen_y as f32) + subpixel_offset.y,
                -self.view_plane_dist
            ).normalize();

            // create ray with direction still in camera space
            let mut ray = Ray {
                origin: match self.projection_mode {
                    CameraProjectionMode::Orthographic => vec3(cam_space_pixel_center.x, cam_space_pixel_center.y, 0.0 ),
                    CameraProjectionMode::Perspective => self.eyepoint,
                },
                direction: match self.projection_mode {
                    CameraProjectionMode::Orthographic => self.view_dir,
                    CameraProjectionMode::Perspective => cam_space_pixel_center
                },
            };

            // rotate ray direction to world space
            let rotation = Matrix3::from_cols(
                self.view_dir.cross(self.up).normalize(),
                self.up,
                -self.view_dir
            );
            ray.direction = rotation * ray.direction;
            rays.push(ray);
        }
        return rays;
    }
}
impl Scene {
    pub fn render_to_image(&self) -> DynamicImage {
        // create image and iterate through its pixels
        let mut img = DynamicImage::new_rgb8(self.camera.screen_width, self.camera.screen_height);
        for x in 0..self.camera.screen_width {
            for y in 0..self.camera.screen_height {
                // get rays, trace, and take average of outputs for AA
                let cam_rays = self.camera.generate_rays(x, y);
                let mut final_color = Vec3::zero();
                for sample_idx in 0..cam_rays.len() {
                    final_color += match self.intersect_ray(&cam_rays[sample_idx]) {
                        None => Vec3::zero(),
                        Some(hit) => {
                            
                            self.phong_shade_hit(&hit)
                        }
                    };
                }
                final_color /= cam_rays.len() as f32;
                
                // convert color to u8 and write to image
                let color_rgb = Rgba::from_channels(
                    (final_color.x as f32 * 255.9999) as u8,
                    (final_color.y as f32 * 255.9999) as u8,
                    (final_color.z as f32 * 255.9999) as u8,
                    0);
                img.put_pixel(x, y, color_rgb);
            }
        }
        return img;
    }
    fn phong_shade_hit(&self, hit: &RayHit) -> Color {
        // standard phong shading
        let to_light = (self.point_light_pos - hit.hitpoint).normalize();
        let to_camera = (self.camera.eyepoint - hit.hitpoint).normalize();
        let reflected = -to_light + 2.0*dot(to_light, hit.normal)*hit.normal;
        let diffuse_weight = (dot(hit.normal, to_light)).clamp(0.0, 1.0);
        let specular_weight = dot(to_camera, reflected).clamp(0.0, 1.0).powf(40.0);
        // cast shadow ray
        let shadow_ray = Ray { origin: hit.hitpoint + 0.01*hit.normal, direction: to_light };
        let shadow_weight = match self.intersect_ray(&shadow_ray) {
            None => 1.0,
            Some(hit) => if hit.distance*hit.distance > (self.point_light_pos - hit.hitpoint).magnitude2() { 1.0 } else { 0.3 }
        };
        return shadow_weight * (self.ambient + diffuse_weight*hit.albedo + specular_weight*vec3(0.4, 0.4, 0.4));
    }
}
impl Intersectable for Sphere {
    fn intersect_ray(&self, ray: &Ray) -> Option<RayHit> {
        // ray-sphere intersection
        let f = ray.origin - self.center;
        let a = ray.direction.magnitude2();
        let b = 2.0*f.dot(ray.direction);
        let c = f.magnitude2() - self.radius*self.radius;
        let d = b*b - 4.0*a*c;
        if d < 0.0 {
            return None;
        }
        else {
            let t = (-b - d.sqrt()) / (2.0*a);
            let hitpoint = ray.origin + t*ray.direction;
            if t < 0.0 { return None; }
            return Some(RayHit {
                distance: t,
                hitpoint: hitpoint,
                normal: (hitpoint - self.center).normalize(),
                albedo: self.albedo,
            })
        }
    }
}
impl Intersectable for Triangle {
    fn intersect_ray(&self, ray: &Ray) -> Option<RayHit> {
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
        if t < 0.0 { return None; }
        return Some(RayHit {
            distance: t,
            hitpoint: hitpoint,
            normal: e1.cross(e2).normalize(),
            albedo: self.albedo,
        })
    }
}
impl Intersectable for Plane {
    fn intersect_ray(&self, ray: &Ray) -> Option<RayHit> {
        // ray-plane intersection
        let to_ray_origin = ray.origin - self.point;
        let origin_dist = dot(to_ray_origin, self.normal);
        let n = origin_dist.signum() * self.normal;
        let d = ray.direction.dot(n);
        if d >= 0.0 { 
            return None;
        }
        else {
            let t = origin_dist.abs() / d.abs();
            let hitpoint = ray.origin + t*ray.direction;
            return Some(RayHit {
                distance: t,
                hitpoint: hitpoint,
                normal: n,
                albedo: self.albedo,
            })
        }
    }
}
impl Intersectable for Scene {
    fn intersect_ray(&self, ray: &Ray) -> Option<RayHit> {
        // for now, just iterate over all intersectables and return shortest (this will probably be a BVH or something later)
        let mut best_hit = None;
        for object in self.objects.iter() {
            if let Some(hit) = object.intersect_ray(ray) {
                best_hit = match best_hit {
                    None => Some(hit),
                    Some(current_best) => {
                        if hit.distance < current_best.distance {
                            Some(hit)
                        }
                        else {
                            Some(current_best)
                        }
                    }
                }
            }
        }
        return best_hit;
    }
}


// FUNCTIONS
pub fn run() {
    // initialize scene
    let scene = Scene {
        camera: Camera {
            eyepoint: Vec3::zero(),
            view_dir: -Vec3::unit_z(),
            up: Vec3::unit_y(),
            view_plane_dist: 0.2,
            pixel_size: 0.001,
            projection_mode: CameraProjectionMode::Perspective,
            screen_width: 400,
            screen_height: 400,
            aa_sample_count: 9,
        },
        objects: vec![
            Box::new(Sphere {
                center: vec3(0.5,0.0,-2.0),
                radius: 0.6,
                albedo: vec3(0.6,0.3,0.3),
            }),
            Box::new(Plane {
                point: vec3(0.0, -2.0, 0.0),
                normal: -Vec3::unit_y(),
                albedo: vec3(0.3,0.6,0.3),
            }),
            Box::new(Triangle {
                a: vec3(-1.4, -0.5, -2.0),
                b: vec3(-0.7, -0.6, -2.0),
                c: vec3(-1.0, 0.5, -2.0),
                albedo: vec3(0.3,0.3,0.6),
            }),
        ],
        point_light_pos: vec3(-1.0,5.0,5.0),
        ambient: vec3(0.1,0.1,0.1),
    };

    // render and write output
    scene.render_to_image().save_with_format("mp1.png", ImageFormat::Png);

}