cmake_minimum_required(VERSION 2.8.3)
project(dalsa_genie_nano_c2420)

FIND_LIBRARY(GEVAPI_LIBRARY GevApi /usr/local/lib/)
FIND_LIBRARY(GCBASE_LIBRARY GCBase_gcc421_v3_0 /opt/genicam_v3_0/bin/Linux64_x64)
#FIND_LIBRARY(X11_LIBRARY X11 /usr/lib/x86_64-linux-gnu/)

find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  image_transport
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #/usr/local/lib
  /usr/dalsa/GigeV/include
  /opt/genicam_v3_0/library/CPP/include/
  #/usr/lib
  #/opt/genicam_v3_0/bin/Linux64_x64/
  /usr/include
)

## Declare a C/C++ library
add_library(GevUtils include/${PROJECT_NAME}/GevUtils.c)
add_library(convertBayer include/${PROJECT_NAME}/convertBayer.c)
add_library(X_Display_utils include/${PROJECT_NAME}/X_Display_utils.c)

## Declare a C++ executable
add_executable(dalsa_genie_nano_c2420_driver src/dalsa_genie_nano_c2420_driver.cpp src/dalsa_genie_nano_c2420_utils.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dalsa_genie_nano_c2420_driver GevUtils convertBayer X_Display_utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GEVAPI_LIBRARY} ${GCBASE_LIBRARY} -lXext) #-lpthread -lXext -lX11 -L/usr/local/lib -lGevApi -lCorW32) #${GENAPI_LIBRARY} ${OpenCV_LIBS}