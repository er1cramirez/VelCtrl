PROJECT(VelCtrl)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

# Add Eigen3 dependency
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

SET(SRC_FILES
	src/main.cpp
	src/VelCtrl.cpp
	src/RobustVelCtrl.cpp
)

# Create main application
FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR VelCtrl
           COPY_RESOURCES
)

# Add test application
SET(TEST_SRC_FILES
	src/TestRobustVelCtrl.cpp
	src/RobustVelCtrl.cpp
)

FLAIR_DEMO(TestRobustVelCtrl "${TEST_SRC_FILES}"
           DEST_DIR VelCtrl
           COPY_RESOURCES
)