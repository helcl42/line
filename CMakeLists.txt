cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

include_directories(src/SvgParser/antlr3/include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(line src/Utils/Timer.cpp)

rosbuild_add_library(line src/Shapes/DetectedObject.cpp)
rosbuild_add_library(line src/Shapes/LinePair.cpp)

rosbuild_add_library(line src/SvgParser/AST/Arguments/Argument.cpp)
rosbuild_add_library(line src/SvgParser/AST/Arguments/Coordinate.cpp)
rosbuild_add_library(line src/SvgParser/AST/Arguments/CoordinatePair.cpp)
rosbuild_add_library(line src/SvgParser/AST/Commands/Command.cpp)
rosbuild_add_library(line src/SvgParser/AST/Commands/CloseCommand.cpp)
rosbuild_add_library(line src/SvgParser/AST/Commands/CurveToCommand.cpp)
rosbuild_add_library(line src/SvgParser/AST/Commands/LineToCommand.cpp)
rosbuild_add_library(line src/SvgParser/AST/Commands/MoveToCommand.cpp)
rosbuild_add_library(line src/SvgParser/AST/Node.cpp)
rosbuild_add_library(line src/SvgParser/AST/PositionFactory.cpp)
rosbuild_add_library(line src/SvgParser/SvgLexer.cpp)
rosbuild_add_library(line src/SvgParser/SvgParser.cpp)

rosbuild_add_library(line src/SvgExtractor/SvgExtractor.cpp)

rosbuild_add_library(line src/XMLParser/XmlParser.cpp)

rosbuild_add_library(line src/Main.cpp)
rosbuild_add_library(line src/DetectionSettings.cpp)
rosbuild_add_library(line src/Line.cpp)
rosbuild_add_library(line src/DetectionParams.cpp)
rosbuild_add_library(line src/ImageService.cpp)
rosbuild_add_library(line src/CameraService.cpp)
rosbuild_add_library(line src/ObjectDetector.cpp)
rosbuild_add_library(line src/LineDetector.cpp)
rosbuild_add_library(line src/SvgObjectDetector.cpp)
rosbuild_add_library(line src/StraightObjectDetector.cpp)
rosbuild_add_executable(line src/DetectorTopic.cpp)

