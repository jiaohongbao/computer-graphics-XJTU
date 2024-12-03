bool Rasterizer::inside_triangle(int x, int y, const Eigen::Vector4f* vertices)
{
    float x1 = vertices[0].x(), y1 = vertices[0].y();
    float x2 = vertices[1].x(), y2 = vertices[1].y();
    float x3 = vertices[2].x(), y3 = vertices[2].y();
    // calculate vector's cross produce
    float CP1 = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1);
    float CP2 = (x3 - x2) * (y - y2) - (y3 - y2) * (x - x2);

    float CP3 = (x1 - x3) * (y - y3) - (y1 - y3) * (x - x3);
    if (CP1 >= 0 && CP2 >= 0 && CP3 >= 0) {
        return true;
    } else {
        return false;
    }
}





std::tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y,
                                                                   const Eigen::Vector4f* v)
{
    Eigen::Matrix<float, 3, 1> P;
    P << 1, x, y;
    Eigen::Matrix<float, 3, 3> Triangle;

    Triangle << 1, 1, 1, v[0].x(), v[1].x(), v[2].x(), v[0].y(), v[1].y(), v[2].y();

    Triangle = Triangle.inverse();

    Eigen::Matrix<float, 3, 1> Answer;

    Answer = Triangle * P;

    return {Answer(0, 0), Answer(1, 0), Answer(2, 0)};
}




void Rasterizer::rasterize_triangle(Triangle& t)
{
  // these lines below are just for compiling and can be deleted
    //(void)t;
    //FragmentShaderPayload payload;
    // these lines above are just for compiling and can be deleted

    // if current pixel is in current triange:
    // 1. interpolate depth(use projection correction algorithm)
    // 2. interpolate vertex positon & normal(use function:interpolate())
    // 3. push primitive into fragment queue
    //
    //
    //
    //FragmentShaderPayload payload;
    






/*
  auto [minx,maxx]=f_min_max(t.viewport_pos[0].x(),t.viewport_pos[1].x(),t.viewport_pos[2].x()); 
    auto [miny,maxy]=f_min_max(t.viewport_pos[0].y(),t.viewport_pos[1].y(),t.viewport_pos[2].y());
 
*/
float minx = std::min({t.viewport_pos[0].x(), t.viewport_pos[1].x(), t.viewport_pos[2].x()});
    float maxx = std::max({t.viewport_pos[0].x(), t.viewport_pos[1].x(), t.viewport_pos[2].x()});
    float miny = std::min({t.viewport_pos[0].y(), t.viewport_pos[1].y(), t.viewport_pos[2].y()});
    float maxy = std::max({t.viewport_pos[0].y(), t.viewport_pos[1].y(), t.viewport_pos[2].y()});
  




    //printf("t.viewport_pos[0].w() = %f, t.viewport_pos[1].w() = %f, t.viewport_pos[2].w() = %f\n",
       //t.viewport_pos[0].w(), t.viewport_pos[1].w(), t.viewport_pos[2].w());
    for (int i = static_cast<int>(std::floor(minx)); i <= static_cast<int>(std::ceil(maxx)); i++) {
        for (int j = static_cast<int>(std::floor(miny)); j <= static_cast<int>(std::ceil(maxy)); j++) {
            FragmentShaderPayload payload;
            
            if (inside_triangle(i, j, t.viewport_pos))
            {


                auto [alpha, beta, gamma] = compute_barycentric_2d(float(i),float(j), t.viewport_pos);


                Vector4f point[3];
                /*
                point[0]<<t.world_pos[0].x(),t.world_pos[0].y(),t.world_pos[0].z(),1;
                point[1]<<t.world_pos[1].x(),t.world_pos[1].y(),t.world_pos[1].z(),1;
                point[2]<<t.world_pos[2].x(),t.world_pos[2].y(),t.world_pos[2].z(),1;
                */
                point[0]=t.viewport_pos[0];
                point[1]=t.viewport_pos[1];
                point[2]=t.viewport_pos[2];


          
                float z_depth_of_point[3];
                z_depth_of_point[0]=t.viewport_pos[0].w();
                z_depth_of_point[1]=t.viewport_pos[1].w();
                z_depth_of_point[2]=t.viewport_pos[2].w();

          
          //auto Zt=1/(alpha/z_depth_of_point[0]+beta/z_depth_of_point[1]+gamma/z_depth_of_point[2]);
          //auto It=(alpha*point[0].z()/z_depth_of_point[0]+beta*point[1].z()/z_depth_of_point[1]+gamma*point[2].z()/z_depth_of_point[2])*Zt;



                float It = alpha * t.viewport_pos[0].z() + beta  * t.viewport_pos[1].z() + gamma * t.viewport_pos[2].z();
                
                
               
                
                payload.world_pos = interpolate(alpha,beta,gamma,t.world_pos[0].head<3>(),t.world_pos[1].head<3>(),t.world_pos[2].head<3>(),{z_depth_of_point[0], z_depth_of_point[1], z_depth_of_point[2]},It);
                payload.world_normal=interpolate(alpha,beta,gamma,t.normal[0],t.normal[1],t.normal[2],{z_depth_of_point[0], z_depth_of_point[1], z_depth_of_point[2]},It);

                
                payload.x=i;
                payload.y=j;
                payload.depth=It;



                std::unique_lock<std::mutex> lock(Context::rasterizer_queue_mutex);
                Context::rasterizer_output_queue.push(payload);
            }
            
        }
    }
}




VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;

    output_payload.viewport_position = Uniforms::MVP * output_payload.world_position;

    Eigen::Matrix<float, 4, 4> factor = Eigen::Matrix4f::Identity();
    factor(0, 0)                      = Uniforms::width / 2.0f;
    factor(1, 1)                      = Uniforms::height / 2.0f;
    factor(0, 3)                      = Uniforms::width / 2.0f;
    factor(1, 3)                      = Uniforms::height / 2.0f;

    output_payload.viewport_position = factor * output_payload.viewport_position;

    Vector3f n = (Uniforms::inv_trans_M.topLeftCorner<3, 3>() * payload.normal).normalized();

    output_payload.normal = n;

    return output_payload;
}

Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, const GL::Material& material,
                               const std::list<Light>& lights, const Camera& camera)
{
    Vector3f ka = material.ambient, kd = material.diffuse, ks = material.specular;

    float shininess = material.shininess;
    Vector3f result;
    result << 0, 0, 0;
    for (auto iterator = lights.begin(); iterator != lights.end(); iterator++) {
        Vector3f direcion_of_camera = (camera.position - payload.world_pos).normalized();
        Vector3f direction_of_light = (iterator->position - payload.world_pos).normalized();

        if (direcion_of_camera.norm() == 0.0f || direction_of_light.norm() == 0.0f) {
            continue;
        }

        Vector3f half = (direction_of_light + direcion_of_camera).normalized();

        float distance = (iterator->position - payload.world_pos).squaredNorm();

        float reducing = iterator->intensity / (distance + 1e-10f);

        float specular_intensity = std::max(half.dot(payload.world_normal.normalized()), 0.0f);

        float diffuse_intensity =
            std::max(direction_of_light.dot(payload.world_normal.normalized()), 0.f);

        Vector3f specular = ks * std::pow(specular_intensity, shininess) * reducing;

        result = ka + kd * diffuse_intensity * reducing + specular;
    }
    //result *= 255.0f;
    result = result.cwiseMax(0.0f).cwiseMin(1.0f) * 255.0f;
    return result;
}


void RasterizerRenderer::render(const Scene& scene)
{
    Uniforms::width       = static_cast<int>(width);
    Uniforms::height      = static_cast<int>(height);
    Context::frame_buffer = FrameBuffer(Uniforms::width, Uniforms::height);
    // clear Color Buffer & Depth Buffer & rendering_res
    Context::frame_buffer.clear(BufferType::Color | BufferType::Depth);
    this->rendering_res.clear();
    // run time statistics
    time_point begin_time                  = steady_clock::now();
    Camera cam                             = scene.camera;
    vertex_processor.vertex_shader_ptr     = vertex_shader;
    fragment_processor.fragment_shader_ptr = phong_fragment_shader;
    for (const auto& group : scene.groups) {
        for (const auto& object : group->objects) {
            Context::vertex_finish     = false;
            Context::rasterizer_finish = false;
            Context::fragment_finish   = false;

            std::vector<std::thread> workers;
            for (int i = 0; i < n_vertex_threads; ++i) {
                workers.emplace_back(&VertexProcessor::worker_thread, &vertex_processor);
            }
            for (int i = 0; i < n_rasterizer_threads; ++i) {
                workers.emplace_back(&Rasterizer::worker_thread, &rasterizer);
            }
            for (int i = 0; i < n_fragment_threads; ++i) {
                workers.emplace_back(&FragmentProcessor::worker_thread, &fragment_processor);
            }

            // set Uniforms for vertex shader
            Uniforms::MVP         = cam.projection() * cam.view() * object->model();
            Uniforms::inv_trans_M = object->model().inverse().transpose();
            Uniforms::width       = static_cast<int>(this->width);
            Uniforms::height      = static_cast<int>(this->height);
            // To do: 同步
            Uniforms::material = object->mesh.material;
            Uniforms::lights   = scene.lights;
            Uniforms::camera   = scene.camera;

            // input object->mesh's vertices & faces & normals data
            const std::vector<float>& vertices     = object->mesh.vertices.data;
            const std::vector<unsigned int>& faces = object->mesh.faces.data;
            const std::vector<float>& normals      = object->mesh.normals.data;
            size_t num_faces                       = faces.size();

            // process vertices
            for (size_t i = 0; i < num_faces; i += 3) {
                for (size_t j = 0; j < 3; j++) {
                    size_t idx = faces[i + j];
                    vertex_processor.input_vertices(
                        Vector4f(vertices[3 * idx], vertices[3 * idx + 1], vertices[3 * idx + 2],
                                 1.0f),
                        Vector3f(normals[3 * idx], normals[3 * idx + 1], normals[3 * idx + 2]));
                }
            }
            vertex_processor.input_vertices(Eigen::Vector4f(0, 0, 0, -1.0f),
                                            Eigen::Vector3f::Zero());
            for (auto& worker : workers) {
                if (worker.joinable()) {
                    worker.join();
                }
            }
        }
    }

    time_point end_time         = steady_clock::now();
    duration rendering_duration = end_time - begin_time;

    this->logger->info("rendering (single thread) takes {:.6f} seconds",
                       rendering_duration.count());

    for (long unsigned int i = 0; i < Context::frame_buffer.depth_buffer.size(); i++) {
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].x()));
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].y()));
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].z()));
    }
}
