Matrix4f Object::model()
{   
  const Quaternionf& r=rotation;
   

  

  auto [x_angle,y_angle,z_angle]=quaternion_to_ZYX_euler(r.w(),r.x(),r.y(),r.z());




  Matrix4f center_matrix,rotation_x_matrix,rotation_z_matrix,rotation_y_matrix,scaling_matrix;

  center_matrix<<1,0,0,center(0,0),
                0,1,0,center(1,0),
                0,0,1,center(2,0),
                0,0,0,1;
  

  scaling_matrix<<scaling(0,0),0,0,0,
                  0,scaling(1,0),0,0,
                  0,0,scaling(2,0),0,
                  0,0,0,1;
  rotation_z_matrix<<cos(radians(z_angle)),-sin(radians(z_angle)),0,0,
                    sin(radians(z_angle)),cos(radians(z_angle)),0,0,
                    0,0,1,0,
                    0,0,0,1;
  rotation_x_matrix<<1,0,0,0,
                    0,cos(radians(x_angle)),-sin(radians(x_angle)),0,
                    0,sin(radians(x_angle)),cos(radians(x_angle)),0,
                    0,0,0,1;
  rotation_y_matrix<<cos(radians(y_angle)),0,sin(radians(y_angle)),0,
                    0,1,0,0,
                    -sin(radians(y_angle)),0,cos(radians(y_angle)),0,
                    0,0,0,1;
   
    Matrix4f R_matrix=rotation_x_matrix*rotation_y_matrix*rotation_z_matrix;

    Matrix4f Identity=scaling_matrix;
    

    Identity=R_matrix*Identity;
    Identity=center_matrix*Identity;





    //Matrix4f Identity=scaling;
    
    /*
    scaling_matrix=rotation_z_matrix*scaling_matrix;
    scaling_matrix=rotation_y_matrix*scaling_matrix;
    scaling_matrix=rotation_x_matrix*scaling_matrix;

    scaling_matrix=center_matrix*scaling_matrix;
      
    */
    

    //center_matrix=center_matrix*



    
    

    //std::cout<<Identity;
    return Identity;
    
    //return Matrix4f::Identity();
}


Matrix4f Camera::projection()
{
    const float fov_y = radians(fov_y_degrees);
    //const float top   = (target - position).norm() * std::tan(fov_y / 2.0f);
    const float top=std::tan(fov_y/2)*near;

    const float right = top * aspect_ratio;

    Matrix4f projection = Matrix4f::Zero();
    // 使用平行投影时，用户并不能从画面上直观地感受到相机的位置，
    // 因而会产生近处物体裁剪过多的错觉。为了产程更好的观察效果，
    // 这里没有使用相机本身的 near 而是取 near = -far 来让相机能看到“背后”的物体。
    /*
    projection(0, 0) = 1.0f / right;
    projection(1, 1) = 1.0f / top;
    projection(2, 2) = -1.0f / far;
    projection(2, 3) = 0.0f;
    projection(3, 3) = 1.0f;



    */
    //near=-far; 
    //
    

    /*
    float left=-right,bottom=-top;
    projection<<2*near/(right-left),0,0,-(right+left)/(right-left),
                0,-2*far/(top-bottom),(top+bottom)/(top-bottom),0,
                0,0,-(far+near)/(far-near),-2*far*near/(far-near),
                0,0,-1,0;

    */
    //float left=-right,bottom=-top;
    projection<<near/right,0,0,0,
                0,near/top,0,0,
                0,0,-(far+near)/(far-near),-2*far*near/(far-near),
                0,0,-1,0;










    return projection;
}


void Toolbar::layout_mode(Scene& scene)
{
    if (ImGui::BeginTabItem("Layout")) {
        if (mode != WorkingMode::LAYOUT) {
            on_selection_canceled();
            mode = WorkingMode::LAYOUT;
        }
        scene_hierarchies(scene);

        Object* selected_object = scene.selected_object;
        if (selected_object != nullptr) {
            material_editor(selected_object->mesh.material);
            ImGui::SeparatorText("Transform");
            ImGui::Text("Translation");
            ImGui::PushID("Translation##");
            Vector3f& center = selected_object->center;
            xyz_drag(&center.x(), &center.y(), &center.z(), POSITION_UNIT);
            ImGui::PopID();

            ImGui::Text("Scaling");
            ImGui::PushID("Scaling##");
            Vector3f& scaling = selected_object->scaling;
            xyz_drag(&scaling.x(), &scaling.y(), &scaling.z(), SCALING_UNIT);
            ImGui::PopID();

            const Quaternionf& r             = selected_object->rotation;
            auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(), r.y(), r.z());
            ImGui::Text("Rotation (ZYX Euler)");
            ImGui::PushID("Rotation##");
            ImGui::PushItemWidth(0.3f * ImGui::CalcItemWidth());
            ImGui::DragFloat("pitch", &x_angle, ANGLE_UNIT, -180.0f, 180.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::SameLine();
            ImGui::DragFloat("yaw", &y_angle, ANGLE_UNIT, -90.0f, 90.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::SameLine();
            ImGui::DragFloat("roll", &z_angle, ANGLE_UNIT, -180.0f, 180.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::PopItemWidth();
            ImGui::PopID();

            /*
            selected_object->rotation = AngleAxisf(radians(x_angle), Vector3f::UnitX()) *
                                        AngleAxisf(radians(y_angle), Vector3f::UnitY()) *
                                        AngleAxisf(radians(z_angle), Vector3f::UnitZ());



            */







            //Eigen::Quaternionf result;

            float cosx,sinx,cosy,siny,cosz,sinz;


            
            cosx=cos(radians(x_angle/2));
            sinx=sin(radians(x_angle/2));
            cosy=cos(radians(y_angle/2));
            siny=sin(radians(y_angle/2));
            cosz=cos(radians(z_angle/2));
            sinz=sin(radians(z_angle/2));
            /*
            cosx=cos(x_angle/2);
            sinx=sin(x_angle/2);
            cosy=cos(y_angle/2);
            siny=sin(y_angle/2);
            cosz=cos(z_angle/2);
            sinz=sin(z_angle/2);
            */


            Eigen::Quaternionf result;
            result.w()=cosx*cosy*cosz-sinx*siny*sinz;
            result.x()=cosx*siny*sinz+sinx*cosy*cosz;
            result.y()=cosx*siny*cosz-sinx*cosy*sinz;
            result.z()=sinx*siny*cosz+cosx*cosy*sinz;

            //Eigen::Quaternionf result(resultw,resultx,resulty,resultz);

            selected_object->rotation=result;



            







        }
        ImGui::EndTabItem();
    }
}
