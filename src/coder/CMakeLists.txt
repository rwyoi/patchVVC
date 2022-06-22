add_library(IO SHARED io.cpp)
target_link_libraries(IO ${PCL_LIBRARIES})

add_library(segment SHARED Patch.cpp SimplePatch.cpp)
target_link_libraries(segment ${PCL_LIBRARIES})

add_library(registration SHARED regist_base.cpp ICP.cpp)
target_link_libraries(registration ${PCL_LIBRARIES})

add_library(octree SHARED GOF.cpp)
target_link_libraries(octree ${PCL_LIBRARIES})

add_library(encoder SHARED encoder.cpp)
target_link_libraries(encoder ${PCL_LIBRARIES} ${VVS_LIBRARIES})