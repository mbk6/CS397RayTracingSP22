// MP1 - based on CS 419 mp1

use std::{ops::Neg, iter::Scan};

// includes
use image::*;
use cgmath::*;

// constants & typedefs
type Vec3 = Vector3<f32>;

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
}
struct Scene {
    pub camera: Camera,
    pub objects: Vec<Box<dyn Intersectable>>,
}
struct Sphere {
    pub center: Vec3,
    pub radius: f32,
}

// IMPL
impl Camera {
    // generate camera ray given pixel coordinates - based on 419 lectures
    pub fn generate_ray(&self, screen_x: u32, screen_y: u32) -> Ray {
        let cam_space_pixel_center = vec3(
            self.pixel_size*(screen_x as f32 - 0.5*(self.screen_width as f32) + 0.5),
            self.pixel_size*(0.5 + 0.5*(self.screen_height as f32) - screen_y as f32),
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
        return ray;
    }
}
impl Scene {
    pub fn render_to_image(&self) -> DynamicImage {
        let mut img = DynamicImage::new_rgb8(self.camera.screen_width, self.camera.screen_height);
        for x in 0..self.camera.screen_width {
            for y in 0..self.camera.screen_height {
                let cam_ray = self.camera.generate_ray(x, y);
                let mut color = match self.intersect_ray(&cam_ray) {
                    None => vec3(0.0, 0.0, 0.0),
                    Some(hit) => hit.normal
                };
                let color_rgb = Rgba::from_channels(
                    (color.x as f32 * 255.9999) as u8,
                    (color.y as f32 * 255.9999) as u8,
                    (color.z as f32 * 255.9999) as u8,
                    0);
                img.put_pixel(x, y, color_rgb);
            }
        }
        return img;
    }
}
impl Intersectable for Sphere {
    fn intersect_ray(&self, ray: &Ray) -> Option<RayHit> {
        // based on 419 lecture on ray-sphere intersection
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
            return Some(RayHit {
                distance: t,
                hitpoint: hitpoint,
                normal: (hitpoint - self.center).normalize(),
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
    let scene = Scene {
        camera: Camera {
            eyepoint: Vec3::zero(),
            view_dir: -Vec3::unit_z(),
            up: Vec3::unit_y(),
            view_plane_dist: 0.1,
            pixel_size: 0.001,
            projection_mode: CameraProjectionMode::Perspective,
            screen_width: 400,
            screen_height: 400,
        },
        objects: vec![
            Box::new(Sphere {
                center: vec3(0.0,0.0,-1.0),
                radius: 0.6,
            })
        ],
    };

    scene.render_to_image().save_with_format("mp1.png", ImageFormat::Png);

}