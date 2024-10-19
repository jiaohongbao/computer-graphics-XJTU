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

    //Eigen::Matrix4f Uniforms::MVP;
    //int Uniforms::width;
    //int Uniforms::height;
    Vector4f clip = Uniforms::MVP*payload.world_position;
    /*
    for(int i=0;i<4;i++){
      clip(i)=clip(i)/clip(3);
    }

    Eigen::Matrix4f a=Eigen::Matrix4f::Identity();
    a(0,0)=Uniforms::width/2;
    a(1,1)=Uniforms::height/2;
    a(0,3)=Uniforms::width/2;
    a(1,3)=Uniforms::height/2;

    output_payload.viewport_position=a*clip;


    Vector4f n(payload.normal(0),payload.normal(1),payload.normal(2),1);
    //Eigen::Matrix4f Uniforms::inv_trans_M;
    n=Uniforms::inv_trans_M*n;
    

    for(int i=0;i<3;i++){
      output_payload.normal(i)=n(i);
    }
    */
    
    output_payload.viewport_position.x()=(clip.x()/clip.w())*(Uniforms::width/2.0f)+(Uniforms::width/2.0f);
    output_payload.viewport_position.y()=(clip.y()/clip.w())*(Uniforms::height/2.0f)+(Uniforms::height/2.0f);
    output_payload.viewport_position.z()=(clip.z()/clip.w());

    output_payload.normal=(Uniforms::inv_trans_M*payload.normal.homogeneous()).head<3>().normalized();

    // Vertex position transformation

    // Viewport transformation

    // Vertex normal transformation

    return output_payload;
}

Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, const GL::Material& material,
                               const std::list<Light>& lights, const Camera& camera)
{
    Eigen::Vector3f ka=material.ambient;
    Eigen::Vector3f kd=material.diffuse;
    Eigen::Vector3f ks=material.specular;

    Eigen::Vector3f point=payload.world_pos,normal=payload.world_normal,eye_pos=camera.position;
    Eigen::Vector3f amb_light_intensity={1,1,1};
    Eigen::Vector3f result_color={0,0,0};

    Eigen::Vector3f La;

    for(int i=0;i<3;i++){
      La(i)=ka(i)*amb_light_intensity(i);
    }
    result_color=result_color+La;

    for(auto& light : lights){
      Eigen::Vector3f lightPos=light.position;

      float r =(lightPos-point).dot(lightPos-point);

      Eigen::Vector3f n=normal.normalized();
      Eigen::Vector3f l=(lightPos-point).normalized();
      Eigen::Vector3f v=(eye_pos-point).normalized();
      Eigen::Vector3f h=(v+l).normalized();
      Eigen::Vector3f Ld=kd*(light.intensity/r)*std::max(0.0f,n.dot(l));
      Eigen::Vector3f Ls=ks*(light.intensity/r)*std::pow(std::max(0.0f,n.dot(h)),material.shininess);

      result_color=result_color+Ld+Ls;

    }
    for(int i=0;i<3;i++){
      if(result_color(i)<0){
        result_color(i)=0;
      }
      if(result_color(i)>1){
        result_color(i)=1;
      }
    }

    /*
    // these lines below are just for compiling and can be deleted
    (void)payload;
    (void)material;
    (void)lights;
    (void)camera;
    // these lines above are just for compiling and can be deleted

    Vector3f result = {0, 0, 0};

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
    */
    return result_color * 255.f;
}
