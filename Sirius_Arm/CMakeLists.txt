include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(/usr/local/include)
include_directories(/opt/openrobots/include)
include_directories(../spi)
set(LINK_DIR /usr/local/lib /opt/openrobots/lib ../spi)
link_directories(${LINK_DIR})

file(GLOB sources "*.cpp" "*.c"  "*.cpp" )

add_library(Sirius_Arm SHARED ${sources} ${LINK_DIR})
target_link_libraries(Sirius_Arm pthread pinocchio)
set_target_properties(Sirius_Arm PROPERTIES LINKER_LANGUAGE CXX)