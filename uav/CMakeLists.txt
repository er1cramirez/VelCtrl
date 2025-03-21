PROJECT(VelCtrl)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

# Include the local Eigen directory
include_directories(
    src/eigen-3.4.0
)

SET(SRC_FILES
	src/main.cpp
	src/VelCtrl.cpp
	src/RobustVelCtrl.cpp
)
SET(EIGEN_TEST_SRC
    src/test_eigen.cpp
)

FLAIR_DEMO(TestEigen "${EIGEN_TEST_SRC}"
           DEST_DIR VelCtrl
           COPY_RESOURCES
)
# Create main application
FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR VelCtrl
           COPY_RESOURCES
)