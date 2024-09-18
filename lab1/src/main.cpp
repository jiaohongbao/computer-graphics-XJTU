#include <cerrno>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<cmath>
#include<spdlog/spdlog.h>
#include<memory>
#include<spdlog/sinks/stdout_color_sinks.h>
#include<spdlog/sinks/basic_file_sink.h>
#include"formatter.hpp"

double function1(Eigen::Matrix<double,2,1> x){
    return x(0,0)*x(0,0)+x(1,0)*x(1,0);
}


double abs(Eigen::Matrix<double,2,1> x){
    return std::sqrt(x(0,0)*x(0,0)+x(1,0)*x(1,0));
}

Eigen::Matrix<double,2,1> grad(Eigen::Matrix<double,2,1> x){
    Eigen::Matrix<double,2,1> GRAD;
    double epslion=1e-5;
    Eigen::Matrix<double,2,1> d_o_x,d_o_y;
    d_o_x(0,0)=x(0,0)+epslion;
    d_o_x(1,0)=x(1,0);
    d_o_y(0,0)=x(0,0);
    d_o_y(1,0)=x(1,0)+epslion;
    double f_dx=function1(d_o_x);
    double f_dy=function1(d_o_y);
    double f=function1(x);
    GRAD(0,0)=(f_dx-f)/epslion;
    GRAD(1,0)=(f_dy-f)/epslion;
    return GRAD;
}

int main(int argc, char* argv[]){
    spdlog::set_level(spdlog::level::debug);
    auto console_sink=std::make_shared<spdlog::sinks::stdout_color_sink_st>();
    auto file_sink=std::make_shared<spdlog::sinks::basic_file_sink_st>("optimizer.log",true);
    spdlog::logger logger("optimizer", {console_sink,file_sink});

    spdlog::set_level(spdlog::level::debug);

    logger.set_level(spdlog::level::debug);
    file_sink->set_level(spdlog::level::debug);

    long long p=2221411116;
    //std::cout<<p;
    double x=p%827;
    //std::cout<<x0;
    double y=p%1709;
    double lambda=0.5;
    double eps=0.01;
    int flag1=0;//flag1=1 mean have found x*
    Eigen::Matrix<double,2,1> x0,xk,xkk;
    x0(0,0)=x;
    x0(1,0)=y;
    int flag2=0;//flag2=1 mean xk have been initialized
    Eigen::Matrix<double,2,2> hessian,hessian_invert;
    hessian(0,0)=2;
    hessian(0,1)=0;
    hessian(1,0)=0;
    hessian(1,1)=2;
    hessian_invert=hessian.inverse();
    while(true){
        if(flag1==1){
            break;
        }
        if(flag2==0){
            flag2=1;
            xk=x0;
            //spdlog::debug("{:.10f}",xk);
            logger.debug("{:.15f}",xk);
            Eigen::Matrix<double,2,1> g=grad(xk);
            xkk=xk-lambda*hessian_invert*g;
            continue;
        }
        else{
            if(abs(xkk-xk)<eps){
                flag1=1;
                continue;
            }
            else{
                xk=xkk;
                Eigen::Matrix<double,2,1> g=grad(xk);
                xkk=xk-lambda*hessian_invert*g;
                //spdlog::debug("{:.10f}",xk);
                logger.debug("{:.15f}",xk);
                continue;
            }
        }

    }
    //spdlog::info("{:.10f}",abs(xkk-xk));
    logger.info("{:.15f}",abs(xkk-xk));
    
    return 0;
}







