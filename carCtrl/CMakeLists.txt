# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(car_ctrl)


SET("OpenCV_DIR" "/home/pi/homescripts/opencv/build")
SET("raspicam_DIR" "/usr/local/lib/cmake/")

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(raspicam REQUIRED)

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

link_directories(/usr/include/)

file(GLOB car_ctrl_SRC
	"*.h"
	"*.c"
	"*.cpp"
	"vichw/*.h",
	"vichw/*.c",
	"vichw/*.cpp",

)

# Declare the executable target built from your sources

set(CMAKE_CXX_FLAGS "-I/usr/local/include -L/usr/local/lib -lpigpio -lrt -lpthread -Wall -std=c++11")
add_executable(car_ctrl ${car_ctrl_SRC})

# Link your application with OpenCV libraries
#target_link_libraries(car_ctrl ${OpenCV_LIBS})

target_link_libraries(car_ctrl ${raspicam_CV_LIBS})
target_link_libraries(car_ctrl ${raspicam_LIBS})
target_link_libraries(car_ctrl ${OpenCV_LIBS})
target_link_libraries(car_ctrl bluetooth )

