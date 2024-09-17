#include<cerrno>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<cmath>
#include<spdlog/spdlog.h>
#include<memory>
#include<spdlog/sinks/stdout_color_sinks.h>
#include<spdlog/sinks/basic_file_sink.h>
#include"formatter.hpp"

double function1(Eigen::Matrix<double,2,1> x);

double abs(Eigen::Matrix<double,2,1> x);

Eigen::Matrix<double,2,1> grad(Eigen::Matrix<double,2,1> x);





