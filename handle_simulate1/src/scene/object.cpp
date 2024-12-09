#include "object.h"

#include <array>
#include <optional>

#ifdef _WIN32
#include <Windows.h>
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fmt/format.h>

#include "../utils/math.hpp"
#include "../utils/ray.h"
#include "../simulation/solver.h"
#include "../utils/logger.h"

using Eigen::Matrix4f;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using std::array;
using std::make_unique;
using std::optional;
using std::string;
using std::vector;

bool Object::BVH_for_collision   = false;
size_t Object::next_available_id = 0;
std::function<KineticState(const KineticState&, const KineticState&)> Object::step =
    forward_euler_step;

Object::Object(const string& object_name)
    : name(object_name), center(0.0f, 0.0f, 0.0f), scaling(1.0f, 1.0f, 1.0f),
      rotation(1.0f, 0.0f, 0.0f, 0.0f), velocity(0.0f, 0.0f, 0.0f), force(0.0f, 0.0f, 0.0f),
      mass(1.0f), BVH_boxes("BVH", GL::Mesh::highlight_wireframe_color)
{
    visible  = true;
    modified = false;
    id       = next_available_id;
    ++next_available_id;
    bvh                      = make_unique<BVH>(mesh);
    const string logger_name = fmt::format("{} (Object ID: {})", name, id);
    logger                   = get_logger(logger_name);
}

Matrix4f Object::model()
{

    const Quaternionf& r = rotation;

    auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(), r.y(), r.z());

    Matrix4f center_matrix, rotation_x_matrix, rotation_z_matrix, rotation_y_matrix, scaling_matrix;

    center_matrix << 1, 0, 0, center(0, 0), 0, 1, 0, center(1, 0), 0, 0, 1, center(2, 0), 0, 0, 0,
        1;

    scaling_matrix << scaling(0, 0), 0, 0, 0, 0, scaling(1, 0), 0, 0, 0, 0, scaling(2, 0), 0, 0, 0,
        0, 1;
    rotation_z_matrix << cos(radians(z_angle)), -sin(radians(z_angle)), 0, 0, sin(radians(z_angle)),
        cos(radians(z_angle)), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    rotation_x_matrix << 1, 0, 0, 0, 0, cos(radians(x_angle)), -sin(radians(x_angle)), 0, 0,
        sin(radians(x_angle)), cos(radians(x_angle)), 0, 0, 0, 0, 1;
    rotation_y_matrix << cos(radians(y_angle)), 0, sin(radians(y_angle)), 0, 0, 1, 0, 0,
        -sin(radians(y_angle)), 0, cos(radians(y_angle)), 0, 0, 0, 0, 1;

    Matrix4f R_matrix = rotation_x_matrix * rotation_y_matrix * rotation_z_matrix;

    Matrix4f Identity = scaling_matrix;

    Identity = R_matrix * Identity;
    Identity = center_matrix * Identity;

    // Matrix4f Identity=scaling;

    /*
    scaling_matrix=rotation_z_matrix*scaling_matrix;
    scaling_matrix=rotation_y_matrix*scaling_matrix;
    scaling_matrix=rotation_x_matrix*scaling_matrix;

    scaling_matrix=center_matrix*scaling_matrix;

    */

    // center_matrix=center_matrix*

    // std::cout<<Identity;
    return Identity;

    // return Matrix4f::Identity();
}

void Object::update(vector<Object*>& all_objects)
{
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    this->center = next_state.position; 

    for (auto* object : all_objects) {
        if (object == this) {
          continue;
        }

        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            auto [v0_idx, v1_idx] = mesh.edge(i);
            Vector3f v0 = (model() * mesh.vertex(v0_idx).homogeneous()).hnormalized();
            Vector3f v1 = (model() * mesh.vertex(v1_idx).homogeneous()).hnormalized();
            
            
            Ray edge_ray{v0, (v1 - v0).normalized()};
            std::optional<Intersection> intersection;

            
            if (BVH_for_collision) {
                intersection = object->bvh->intersect(edge_ray, object->mesh, object->model());
            } else {
                intersection = naive_intersect(edge_ray, object->mesh, object->model());
            }
        if (intersection && intersection->t <= (v1 - v0).norm()) {
                
                next_state.position = current_state.position;

                
                Vector3f relative_velocity = next_state.velocity - object->velocity;
                float impulse = -2.0f * relative_velocity.dot(intersection->normal) / 
                                (1.0f / mass + 1.0f / object->mass);

                
                next_state.velocity += (impulse / mass) * intersection->normal;
                object->velocity -= (impulse / object->mass) * intersection->normal;

                break;
            }
        }
    }

    
    center = next_state.position;
    velocity = next_state.velocity;
    force = next_state.acceleration * mass;
    prev_state = current_state;
}

void Object::render(const Shader& shader, WorkingMode mode, bool selected)
{
    if (modified) {
        mesh.VAO.bind();
        mesh.vertices.to_gpu();
        mesh.normals.to_gpu();
        mesh.edges.to_gpu();
        mesh.edges.release();
        mesh.faces.to_gpu();
        mesh.faces.release();
        mesh.VAO.release();
    }
    modified = false;
    // Render faces anyway.
    unsigned int element_flags = GL::Mesh::faces_flag;
    if (mode == WorkingMode::MODEL) {
        // For *Model* mode, only the selected object is rendered at the center in the world.
        // So the model transform is the identity matrix.
        shader.set_uniform("model", I4f);
        shader.set_uniform("normal_transform", I4f);
        element_flags |= GL::Mesh::vertices_flag;
        element_flags |= GL::Mesh::edges_flag;
    } else {
        Matrix4f model = this->model();
        shader.set_uniform("model", model);
        shader.set_uniform("normal_transform", (Matrix4f)(model.inverse().transpose()));
    }
    // Render edges of the selected object for modes with picking enabled.
    if (check_picking_enabled(mode) && selected) {
        element_flags |= GL::Mesh::edges_flag;
    }
    mesh.render(shader, element_flags);
}

void Object::rebuild_BVH()
{
    bvh->recursively_delete(bvh->root);
    bvh->build();
    BVH_boxes.clear();
    refresh_BVH_boxes(bvh->root);
    BVH_boxes.to_gpu();
}

void Object::refresh_BVH_boxes(BVHNode* node)
{
    if (node == nullptr) {
        return;
    }
    BVH_boxes.add_AABB(node->aabb.p_min, node->aabb.p_max);
    refresh_BVH_boxes(node->left);
    refresh_BVH_boxes(node->right);
}
