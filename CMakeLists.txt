cmake_minimum_required(VERSION 3.4)
set(PROJECT_NAME slam-Testbed)
project(slam-Testbed)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(Eigen Statistics Properties transformation)

aux_source_directory(./ SRC_LIST)
aux_source_directory(./Properties Src_properties)
aux_source_directory(./Statistics Src_stat)
aux_source_directory(./transformation Src_transform)
aux_source_directory(./Interpolator Src_interpolate)
aux_source_directory(./Registrador Src_registrador)
aux_source_directory(./GeneratorPCA Src_pca)
aux_source_directory(./ModuloEscala Src_findscala)
aux_source_directory(./AjusteTiempo Src_adjuste)
#file(GLOB_RECURSE SRCS *.cpp)
#file(GLOB_RECURSE HDRS *.h)
#file(GLOB_RECURSE UIRS *.ui)

add_executable(slam-Testbed ${SRC_LIST} ${Src_properties} ${Src_stat} ${Src_transform} ${Src_interpolate} ${Src_registrador} ${Src_pca} ${Src_findscala} ${Src_adjuste})
#add_executable(slam-Testbed_refactor ${SRCS} ${HDRS} ${UIRS})

target_link_libraries(slam-Testbed Qt5::Core)
target_link_libraries(slam-Testbed Qt5::Widgets)
target_link_libraries(slam-Testbed ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})