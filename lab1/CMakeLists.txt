cmake_minimum_required(VERSION 3.11)




project(optimizer VERSION 0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(SOURCES
    src/main.cpp
    )


add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME}
    PRIVATE deps)


target_compile_definitions(${PROJECT_NAME}
    PRIVATE SPDLOG_FMT_EXTERNAL
    PRIVATE FMT_HEADER_ONLY)
#find_package(fmt REQUIRED)
add_subdirectory(deps/fmt)
target_link_libraries(${PROJECT_NAME} fmt::fmt)


target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    )











