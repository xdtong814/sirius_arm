include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(/usr/local/include)
include_directories(/opt/openrobots/include)
set(LINK_DIR /usr/local/lib /opt/openrobots/lib)
link_directories(${LINK_DIR})

file(GLOB sources "*.cpp" "*.c"  "*.cpp" )

add_library(spi SHARED ${sources} ${LINK_DIR})
target_link_libraries(spi pthread)
set_target_properties(spi PROPERTIES LINKER_LANGUAGE CXX)