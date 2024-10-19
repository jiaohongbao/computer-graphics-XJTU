#include "rasterizer_renderer.h"
#include "../utils/math.hpp"
#include <cstdio>

#ifdef _WIN32
#undef min
#undef max
#endif

using Eigen::Vector3f;
using Eigen::Vector4f;

// vertex shader
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;

    // 1. 顶点位置变换到裁剪空间
    Eigen::Vector4f clip_position = Uniforms::MVP * payload.world_position;

    // 2. 进行视口变换
    output_payload.viewport_position.x() = (clip_position.x() / clip_position.w()) * (Uniforms::width / 2.0f) + (Uniforms::width / 2.0f);
    output_payload.viewport_position.y() = (clip_position.y() / clip_position.w()) * (Uniforms::height / 2.0f) + (Uniforms::height / 2.0f);
    output_payload.viewport_position.z() = clip_position.z() / clip_position.w(); // 深度值

    // 3. 法线向量变换到相机坐标系
    output_payload.normal = (Uniforms::inv_trans_M * payload.normal.homogeneous()).head<3>().normalized(); // 归一化法线

    return output_payload;
}


Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, const GL::Material& material,
                               const std::list<Light>& lights, const Camera& camera)
{
    Vector3f result = {0, 0, 0};

    // 材质属性
    Vector3f ka = material.ambient; // 环境光系数
    Vector3f kd = material.diffuse;  // 漫反射系数
    Vector3f ks = material.specular; // 镜面反射系数
    float shininess = material.shininess;

    // 遍历所有光源
    for (const auto& light : lights)
    {
        // 计算光源方向
        Vector3f light_dir = (light.position - payload.world_pos).normalized();//此时payload.world_po为nan
        if (light_dir.norm() == 0.0f) continue; // 跳过光源方向为零的情况nan
        //printf("payload.world_pos = %f, %f, %f\n", payload.world_pos.x(), payload.world_pos.y(), payload.world_pos.z());
        // 视线方向
        Vector3f view_dir = (camera.position - payload.world_pos).normalized();
        if (view_dir.norm() == 0.0f) continue; // 跳过视线方向为零的情况nan

        // 计算半向量nan
        Vector3f half_vector = (light_dir + view_dir).normalized();

        
        // 计算光照衰减
        float distance = (light.position - payload.world_pos).squaredNorm();
        float attenuation = light.intensity/distance;

        // 计算环境光分量
        Vector3f ambient = ka * light.intensity;

        // 计算漫反射分量nan
        float diff = std::max(light_dir.dot(payload.world_normal.normalized()), 0.0f); // L·N 计算
        Vector3f diffuse = kd  * diff * attenuation;

        // 计算镜面反射分量nan
        float spec = std::max(half_vector.dot(payload.world_normal.normalized()), 0.0f); // H·N 计算
        Vector3f specular = ks  * std::pow(spec, shininess) * attenuation;

        // 将所有光照分量累加到结果中
        result += ambient + diffuse + specular;
        //printf("light_dir = %f, %f, %f\n", light_dir.x(), light_dir.y(), light_dir.z());
       //printf("view_dir = %f, %f, %f\n", view_dir.x(), view_dir.y(), view_dir.z());
        //printf("half_vector = %f, %f, %f\n", half_vector.x(), half_vector.y(), half_vector.z());
        //printf("ambient = %f, %f, %f\n", ambient.x(), ambient.y(), ambient.z());
       // printf("diffuse = %f, %f, %f\n", diffuse.x(), diffuse.y(), diffuse.z());
        //printf("specular = %f, %f, %f\n", specular.x(), specular.y(), specular.z());
    }

    // 确保结果在[0, 1]范围内
    //result = result.cwiseMax(0.0f).cwiseMin(1.0f);

    // 设置渲染结果的最大阈值为255
    result = result * 255.0f;

    //printf("result = %f,%f,%f\n",result.x(),result.y(),result.z());
    return result;
}



    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular

    // set ambient light intensity

    // Light Direction

    // View Direction

    // Half Vector

    // Light Attenuation

    // Ambient

    // Diffuse

    // Specular

    // set rendering result max threshold to 255