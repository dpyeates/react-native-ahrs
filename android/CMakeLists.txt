cmake_minimum_required(VERSION 3.13)

project(ahrs)

# Set the path to node_modules (can be overridden by parent project)
if(NOT DEFINED NODE_MODULES_DIR)
    # First try the example structure (for development)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../node_modules")
        set(NODE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../node_modules")
    # Then try the typical React Native app structure (when installed as dependency)
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../node_modules")
        set(NODE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../node_modules")
    # Fallback for other structures
    else()
        set(NODE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../node_modules")
    endif()
endif()

# Path to React Native
set(RN_DIR "${NODE_MODULES_DIR}/react-native")

# Find log library
find_library(
    log-lib
    log
)

# Eigen is included directly in the repository
# Headers are at ../fusionml/src/Eigen

# Create the shared library
add_library(ahrs SHARED
    jni/AhrsModuleJNI.cpp
    ../fusionml/src/uNavINS.cpp
    ../fusionml/src/AltitudeCalculator.cpp
    ../fusionml/src/FlightPhaseDetector.cpp
    ../fusionml/src/JsonRecorder.cpp
)

# Specify include directories
target_include_directories(
    ahrs
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../fusionml/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../fusionml/src/Eigen"
    "${RN_DIR}/ReactCommon/jsi"
    "${RN_DIR}/ReactCommon"
    "${RN_DIR}/ReactAndroid/src/main/jni"
    "${RN_DIR}/ReactAndroid/src/main/java/com/facebook/react/turbomodule/core/jni"
)

# Link libraries
target_link_libraries(ahrs
    ${log-lib}
    android
    z  # zlib for gzip compression
)

if(ReactAndroid_VERSION_MINOR GREATER_EQUAL 81)
  target_compile_reactnative_options(ahrs PRIVATE)
endif()



