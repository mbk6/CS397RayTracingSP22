use std::borrow::BorrowMut;

use tobj::{self, Mesh};
use cgmath::*;

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


pub fn run() {
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
    
    // compute normals if not present
    let teapot = (models[0].mesh).borrow_mut();
    if teapot.normals.is_empty() {
        compute_normals(teapot);
    }
    assert!(!teapot.normals.is_empty());

    
}