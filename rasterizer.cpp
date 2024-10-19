#include <array>
#include <limits>
#include <tuple>
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "rasterizer.h"
#include "triangle.h"
#include "../utils/math.hpp"

using Eigen::Matrix4f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::fill;
using std::tuple;

void Rasterizer::worker_thread()
{
    while (true) {
        VertexShaderPayload payload;
        Triangle triangle;
        {
            // printf("vertex_finish = %d\n vertex_shader_output_queue.size = %ld\n",
            // Context::vertex_finish, Context::vertex_shader_output_queue.size());
            if (Context::vertex_finish && Context::vertex_shader_output_queue.empty()) {
                Context::rasterizer_finish = true;
                return;
            }
            if (Context::vertex_shader_output_queue.size() < 3) {
                continue;
            }
            std::unique_lock<std::mutex> lock(Context::vertex_queue_mutex);
            if (Context::vertex_shader_output_queue.size() < 3) {
                continue;
            }
            for (size_t vertex_count = 0; vertex_count < 3; vertex_count++) {
                payload = Context::vertex_shader_output_queue.front();
                Context::vertex_shader_output_queue.pop();
                if (vertex_count == 0) {
                    triangle.world_pos[0]    = payload.world_position;
                    triangle.viewport_pos[0] = payload.viewport_position;
                    triangle.normal[0]       = payload.normal;
                } else if (vertex_count == 1) {
                    triangle.world_pos[1]    = payload.world_position;
                    triangle.viewport_pos[1] = payload.viewport_position;
                    triangle.normal[1]       = payload.normal;
                } else {
                    triangle.world_pos[2]    = payload.world_position;
                    triangle.viewport_pos[2] = payload.viewport_position;
                    triangle.normal[2]       = payload.normal;
                }
            }
        }
        rasterize_triangle(triangle);
    }
}

float sign(Eigen::Vector2f p1, Eigen::Vector2f p2, Eigen::Vector2f p3)
{
    return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，判断(x,y)是否在三角形的内部
bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{


  
    /*Vector3f v[3];
    for (int i = 0; i < 3; i++) v[i] = {vertices[i].x(), vertices[i].y(), 1.0};

    Vector3f p(float(x), float(y), 1.0f);
    */
    //return false;
    //
    
    auto x1=vertices[0].x(),x2=vertices[1].x(),x3=vertices[2].x(),y1=vertices[0].y(),y2=vertices[1].y(),y3=vertices[2].y();
    auto area_abc=0.5*(x1*y2+x2*y3+x3*y1-x3*y2-x2*y1-x1*y3);
    auto area_pbc=0.5*(x*y2+x2*y3+x3*y-x3*y2-x2*y-x*y3);
    auto area_apc=0.5*(x1*y+x*y3+x3*y1-x3*y-x*y1-x1*y3);
    auto area_abp=0.5*(x1*y2+x2*y+x*y1-x*y2-x2*y1-x1*y);

    if(area_abc==area_abp+area_apc+area_pbc){
      return true;
    }
    else{
      return false;
    }
  
    





    
} 

// 给定坐标(x,y)以及三角形的三个顶点坐标，计算(x,y)对应的重心坐标[alpha, beta, gamma]
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{
    float c1 = 0.f, c2 = 0.f, c3 = 0.f;
    /*
    // these lines below are just for compiling and can be deleted
    (void)x;
    (void)y;
    (void)v;
    // these lines above are just for compiling and can be deleted
*/

    



    
    Eigen::Matrix<float,3,3> M;
    M<<1,1,1,
      v[0].x(),v[1].x(),v[2].x(),
      v[0].y(),v[1].y(),v[2].y();
    Eigen::Matrix<float,3,1> X;
    Eigen::Matrix<float,3,1> B;
    B<<1,x,y;
    M=M.inverse();
    X=M*B;
    c1=X(0,0);
    c2=X(1,0);
    c3=X(2,0);
    return {c1, c2, c3};


   
}

// 对顶点的某一属性插值
Vector3f Rasterizer::interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1,
                                 const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3,
                                 const Eigen::Vector3f& weight, const float& Z)
{
    Vector3f interpolated_res;
    for (int i = 0; i < 3; i++) {
        interpolated_res[i] = alpha * vert1[i] / weight[0] + beta * vert2[i] / weight[1] +
                              gamma * vert3[i] / weight[2];
    }
    interpolated_res *= Z;
    return interpolated_res;
}

// 对当前三角形进行光栅化
//
//

tuple<float,float> f_min_max(float a,float b,float c){
  float min=a,max=a;
  if(b<min){
    min=b;
  }
  if(c<min){
    min=c;
  }
  if(b>max){
    max=b;
  }
  if(c>max){
    max=c;
  }
  return{min,max};
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
    auto [minx,maxx]=f_min_max(t.viewport_pos[0].x(),t.viewport_pos[1].x(),t.viewport_pos[2].x()); 
    auto [miny,maxy]=f_min_max(t.viewport_pos[0].y(),t.viewport_pos[1].y(),t.viewport_pos[2].y());
    //printf("t.viewport_pos[0].w() = %f, t.viewport_pos[1].w() = %f, t.viewport_pos[2].w() = %f\n",
       //t.viewport_pos[0].w(), t.viewport_pos[1].w(), t.viewport_pos[2].w());------这里也有问题，全部都是0；

    // 遍历扫描区域内的每一个像素
    for (int i = static_cast<int>(std::floor(minx)); i <= static_cast<int>(std::ceil(maxx)); i++) {
        for (int j = static_cast<int>(std::floor(miny)); j <= static_cast<int>(std::ceil(maxy)); j++) {
            FragmentShaderPayload payload;
            // 判断该像素是否在三角形内
            if (inside_triangle(float(i) , float(j) , t.viewport_pos))
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
