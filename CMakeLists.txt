cmake_minimum_required(VERSION 3.5)
project(spi_com)
set(CMAKE_CXX_STANDARD 17)
include_directories(/home/lingwei/CLionProjects/spi_com/Sirus_Arm)
include_directories(/home/lingwei/CLionProjects/spi_com/spi)
include_directories(/home/lingwei/CLionProjects/spi_com/urdf)
include_directories(/usr/local/include/eigen3)
include_directories(/usr/local/include/)
include_directories(/opt/openrobots/include/)

add_subdirectory(spi)
add_subdirectory(Sirius_Arm)

set(LINK_DIR /usr/local/lib /opt/openrobots/lib)
link_directories(${LINK_DIR})
file(GLOB_RECURSE sources "Sirius_Arm/*.cpp" "spi/*.cpp")

add_executable(spi_com main.cpp ${sources})
target_link_libraries(spi_com spi Sirius_Arm)