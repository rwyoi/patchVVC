add_library(encoder SHARED encoder.cpp)
target_link_libraries(encoder ${PCL_LIBRARIES} ${VVS_LIBRARIES} -lzstd -lturbojpeg)

add_library(decoder SHARED decoder.cpp)
target_link_libraries(decoder ${PCL_LIBRARIES} ${VVS_LIBRARIES} -lzstd -lturbojpeg)