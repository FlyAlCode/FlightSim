include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/include/imu/)

set(SOURCE_FILES ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/src/imu/cIMUCorrect.cpp)
set(SOURCE_FILES ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/src/imu/cIMUOdo.cpp)
set(SOURCE_FILES ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/src/imu/cIMUState.cpp)

set(SOURCE_FILES ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/src/mrsid/sid.cc)
set(SOURCE_FILES ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/src/shp/shp.cc)

set(SOURCE_FILES ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/src/camera.cc)
set(SOURCE_FILES ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/src/ground.cc)
set(SOURCE_FILES ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/src/main.cc)


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# # test 
# # create sat image sequence only
# add_executable(sat_only ${SOURCE_FILES}  ${PROJECT_SOURCE_DIR}/src/take_sat_img.cc)
# TARGET_LINK_LIBRARIES(sat_only ${REQUIRED_LIBRARIES})

# test sid
add_executable(test_sid ${PROJECT_SOURCE_DIR}/src/test/test_sid.cc ${PROJECT_SOURCE_DIR}/src/mrsid/sid.cc)
TARGET_LINK_LIBRARIES(test_sid ${REQUIRED_LIBRARIES})

# test ground
add_executable(test_ground ${PROJECT_SOURCE_DIR}/src/test/test_ground.cc ${PROJECT_SOURCE_DIR}/src/mrsid/sid.cc 
            ${PROJECT_SOURCE_DIR}/src/shp/shp.cc ${PROJECT_SOURCE_DIR}/src/ground.cc)
TARGET_LINK_LIBRARIES(test_ground ${REQUIRED_LIBRARIES})

# test shp
add_executable(test_shp ${PROJECT_SOURCE_DIR}/src/test/test_shp.cc ${PROJECT_SOURCE_DIR}/src/shp/shp.cc)
TARGET_LINK_LIBRARIES(test_shp ${REQUIRED_LIBRARIES})

add_executable(flight_sim ${SOURCE_FILES} )
TARGET_LINK_LIBRARIES(flight_sim ${REQUIRED_LIBRARIES})