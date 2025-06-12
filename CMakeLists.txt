cmake_minimum_required(VERSION 3.5)
project(minimuas-apps)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Locate packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(NDN_SVS REQUIRED libndn-svs)
pkg_check_modules(NDN_CXX REQUIRED libndn-cxx)
pkg_check_modules(NDNSD REQUIRED ndnsd)
pkg_check_modules(NAC_ABE REQUIRED libnac-abe)
pkg_check_modules(OPENCV REQUIRED opencv4)
find_package(MAVSDK REQUIRED)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
pkg_check_modules(NDNSF REQUIRED libndn-service-framework)
pkg_check_modules(PROTOBUF REQUIRED protobuf)

find_package(Boost REQUIRED COMPONENTS chrono)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Boost libraries not found")
endif()

# Helper: glob generated source files
file(GLOB GENERATED_CPP src/generated/*.cpp)
file(GLOB GENERATED_CC src/generated/*.cc)
set(GENERATED_SRCS ${GENERATED_CPP} ${GENERATED_CC})

# Common include dirs
set(COMMON_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/generated
  ${Boost_INCLUDE_DIRS}
)

# gcs-shell
add_executable(gcs-shell src/minimuas_GCS_shell.cpp ${GENERATED_SRCS})
target_include_directories(gcs-shell PRIVATE ${COMMON_INCLUDES} ${OPENCV_INCLUDE_DIRS})
target_link_libraries(gcs-shell PRIVATE ndn-service-framework gtkmm-3.0 MAVSDK::mavsdk ndnsd ndn-cxx ndn-svs protobuf nac-abe ${Boost_LIBRARIES})

# gcs
add_executable(gcs src/minimuas_GCS.cpp ${GENERATED_SRCS})
target_include_directories(gcs PRIVATE ${COMMON_INCLUDES} ${OPENCV_INCLUDE_DIRS})
target_link_libraries(gcs PRIVATE ndn-service-framework gtkmm-3.0 MAVSDK::mavsdk ndnsd ndn-cxx ndn-svs protobuf nac-abe ${Boost_LIBRARIES})

# iuas
add_executable(iuas src/minimuas_IUAS.cpp ${GENERATED_SRCS})
target_include_directories(iuas PRIVATE ${COMMON_INCLUDES} ${OPENCV_INCLUDE_DIRS})
target_link_libraries(iuas PRIVATE ndn-service-framework gtkmm-3.0 MAVSDK::mavsdk ndnsd ndn-cxx ndn-svs protobuf nac-abe ${Boost_LIBRARIES})

# wuas
add_executable(wuas src/minimuas_WUAS.cpp ${GENERATED_SRCS})
target_include_directories(wuas PRIVATE ${COMMON_INCLUDES})
target_link_libraries(wuas PRIVATE ndn-service-framework gtkmm-3.0 MAVSDK::mavsdk ndnsd ndn-cxx ndn-svs protobuf nac-abe ${Boost_LIBRARIES})

# service-controller
add_executable(service-controller src/service-controller.cpp ${GENERATED_SRCS})
target_include_directories(service-controller PRIVATE ${COMMON_INCLUDES})
target_link_libraries(service-controller PRIVATE ndn-service-framework gtkmm-3.0 MAVSDK::mavsdk ndnsd ndn-cxx ndn-svs protobuf nac-abe ${Boost_LIBRARIES})
