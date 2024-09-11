#include<memory>
#include"formatter.hpp"
#include<spdlog/spdlog.h>
#include<spdlog/sinks/stdout_color_sinks.h>
#include<spdlog/sinks/basic_file_sink.h>


//spdlog::set_level(spdlog::level::debug);
//logger->set_level(spdlog::level::debug);
//sink->set_level(spdlog::level:debug);
int main(){
    auto console_sink=std::make_shared<spdlog::sinks::stdout_color_sink_st>();
    auto file_sink=std::make_shared<spdlog::sinks::basic_file_sink_st>("my_program.log",true);

    spdlog::logger my_logger("my logger",{console_sink,file_sink});
    my_logger.debug("This is a debug message");
    my_logger.info("Some information during processing");
    my_logger.warn("Warning,something is going wrong");
    my_logger.error("An error occurred");
    my_logger.critical("Critical error, emergency stop");

    //auto logger=spdlog::sinks::stdout_color_sink_mt();
    auto logger=spdlog::stdout_color_st("my_logger");
    int a=10;
    float pi=3.1415926;
    std::string message="Hello,world!";

    logger->info("a integer: {}",a);
    logger->info("my program says: {}",message);
    logger->info("pi is {},approximation: {:.2f}",pi,pi);
    return 0;
}
