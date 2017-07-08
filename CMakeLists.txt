#Specify minimum cmake requirements
cmake_minimum_required(VERSION 2.8.12)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0008 NEW)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#工程名
project(MobileManipulate)
#添加的VISP配置
find_package(VISP REQUIRED)

# Pull in standard configuration.
include(../CytonBuildDefaults.cmake)

# Pull in rendering requirements
include_directories(${render_inc})
#添加的VISP配置
include_directories(${VISP_INCLUDE_DIRS})

link_directories(${render_lib})
link_directories("${toolkits}/../lib")

# Set the contents of our executable
#添加.cpp和.h文件
add_executable(MobileManipulate 
   main.cpp
   MmArmBase.cpp
   MmArmBase.h
   MmAvoidObstacle.cpp
   MmAvoidObstacle.h
   MmData.cpp
   MmData.h
   MmMobileRobotBase.cpp
   MmMobileRobotBase.h
   MmRFIDAlgorithm.cpp 
   MmRFIDAlgorithm.h      
   MmRFIDBase.cpp
   MmRFIDBase.h           
   MmSocketSrv.cpp
   MmSocketSrv.h
   MmVisualServoAlgorithm.cpp   
   MmVisualServoAlgorithm.h
   MmVisualServoBase.cpp
   MmVisualServoBase.h
)

# Link appropriate libraries

target_link_libraries(
   MobileManipulate
   ecFoundCore
   ecXml
   ecXmlReaderWriter
   ecManipulation
   ecManipulationDirector
   ecSystemSimulation
   ecSimulation
   ecManipulator
   ecFoundCommon
   ecGeometry
   ecControl
   ecPathPlanning
   ecMatrixUtilities
   ecControlCore
   ecFunction
   ecPlugins
   ecboost_iostreams-mt
   ecboost_filesystem-mt
   ecboost_program_options-mt
   ecboost_system-mt
   ecboost_thread-mt   
   ecboost_date_time-mt
   ecRemoteCommand
   #opencv3
   opencv_aruco310
   opencv_bgsegm310
   opencv_bioinspired310
   opencv_calib3d310
   opencv_ccalib310
   opencv_core310
   opencv_datasets310
   opencv_dnn310
   opencv_dpm310
   opencv_face310
   opencv_features2d310
   opencv_flann310
   opencv_fuzzy310
   opencv_highgui310
   opencv_imgcodecs310
   opencv_imgproc310
   opencv_line_descriptor310
   opencv_ml310
   opencv_objdetect310
   opencv_optflow310
   opencv_photo310
   opencv_plot310
   opencv_reg310
   opencv_rgbd310
   opencv_saliency310
   opencv_shape310
   opencv_stereo310
   opencv_stitching310
   opencv_structured_light310
   opencv_superres310
   opencv_surface_matching310
   opencv_text310
   opencv_tracking310
   opencv_ts310
   opencv_video310
   opencv_videoio310
   opencv_videostab310
   opencv_xfeatures2d310
   opencv_ximgproc310
   opencv_xobjdetect310
   opencv_xphoto310
   ippicvmt
   #Basler相机
   GCBase_MD_VC120_v3_0_Basler_pylon_v5_0
   GenApi_MD_VC120_v3_0_Basler_pylon_v5_0
   PylonBase_MD_VC120_v5_0
   PylonC_MD_VC120
   PylonGUI_MD_VC120_v5_0
   PylonUtility_MD_VC120_v5_0
   #VISP配置
   ${VISP_LIBRARIES}
   #先锋机器人
   #AriaVC12.lib
   #ArNetworkingVC12.lib
)

# Link appropriate libraries
if(UNIX)
target_link_libraries(
   MobileManipulate
   "${toolkits}/../bin/remoteCommandClientPlugin.ecp"
   "${toolkits}/../bin/imageSensor.ecp"
)

endif(UNIX)

if(WIN32)
target_link_libraries(
   MobileManipulate
   "${toolkits}/../lib/remoteCommandClientPlugin.ecp.lib"
)
endif(WIN32)

set(CMAKE_BUILD_TYPE Release)