add_library(IO SHARED io.cpp)
target_link_libraries(IO ${PCL_LIBRARIES})

add_library(segment SHARED Patch.cpp SimplePatch.cpp)
target_link_libraries(segment ${PCL_LIBRARIES})

add_library(registration SHARED regist_base.cpp ICP.cpp)
target_link_libraries(registration ${PCL_LIBRARIES})

add_library(octree SHARED deoctree.cpp octree.cpp GOF.cpp)
target_link_libraries(octree ${PCL_LIBRARIES} -lsegment -lregistration -lzstd -lturbojpeg)
