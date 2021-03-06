link_directories(${LIBRARY_OUTPUT_PATH})

# examples
add_executable(simple ${AR_SOURCE_DIR}/sample/simple/main.cpp)
target_link_libraries(simple ARToolKitPlus)

add_executable(multi ${AR_SOURCE_DIR}/sample/multi/main.cpp)
target_link_libraries(multi ARToolKitPlus)
IF(OpenCV_FOUND)
  add_executable(multiOpenCV ${AR_SOURCE_DIR}/sample/multi/mainOpenCV.cpp)
  target_link_libraries(multiOpenCV ARToolKitPlus ${OpenCV_LIBS})

  add_executable(mown ${AR_SOURCE_DIR}/sample/multi/mainOpenCVOwn.cpp)
  target_link_libraries(mown ARToolKitPlus ${OpenCV_LIBS})

  add_executable(mplanar ${AR_SOURCE_DIR}/sample/multi/mainplanar.cpp)
  target_link_libraries(mplanar ARToolKitPlus ${OpenCV_LIBS})
ENDIF(OpenCV_FOUND)

# tools
add_executable(patt2ppm ${AR_SOURCE_DIR}/tools/patt2ppm/main.cpp)
target_link_libraries(patt2ppm ARToolKitPlus)

add_executable(ppm2patt ${AR_SOURCE_DIR}/tools/ppm2patt/main.cpp)
target_link_libraries(ppm2patt ARToolKitPlus)

set(PAT_GEN_DIR ${AR_SOURCE_DIR}/tools/IdPatGen)
add_executable(idpatgen ${PAT_GEN_DIR}/main.cpp ${PAT_GEN_DIR}/PN/Image.cpp ${PAT_GEN_DIR}/PN/ImageTool.cpp)
target_link_libraries(idpatgen ARToolKitPlus)

# test program
add_executable(test ${AR_SOURCE_DIR}/tools/test/main.cpp)
target_link_libraries(test ARToolKitPlus)
