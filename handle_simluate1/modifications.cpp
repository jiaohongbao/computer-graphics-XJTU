

KineticState forward_euler_step([[maybe_unused]] const KineticState& previous,
                                const KineticState& current)
{
    KineticState next;
    next.acceleration = current.acceleration;
    next.velocity     = current.velocity + next.acceleration * time_step;
    next.position     = current.position + next.velocity * time_step;

    return next;
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



void Scene::simulation_update()
{

    auto current_time   = steady_clock::now();
    auto remaining_time = duration(current_time - last_update).count();
    while (remaining_time >= time_step) {
        for (auto& object : all_objects) {
            object->update(all_objects);
        }
        remaining_time = remaining_time - time_step;
    }
    last_update =
        current_time - duration_cast<decltype(last_update)::duration>(duration(remaining_time));
}
