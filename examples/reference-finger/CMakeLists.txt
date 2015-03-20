cmake_minimum_required(VERSION 2.8)

project(RepeatedLineTracking.Example.ReferenceFinger)

file(GLOB_RECURSE RepeatedLineTracking.Example.ReferenceFinger.Sources "src/*.cpp")

add_executable(${PROJECT_NAME} ${RepeatedLineTracking.Example.ReferenceFinger.Sources})

target_link_libraries(${PROJECT_NAME} RepeatedLineTracking)


