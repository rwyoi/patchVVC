add_library(encoder SHARED encoder.cpp)
target_link_libraries(encoder ${PCL_LIBRARIES} ${VVS_LIBRARIES})