project(elmapd)

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

set(DEV ON)

IF(DEV)
    message(STATUS "Dev mode is enabled")
    add_definitions(-DDEV)
ENDIF()

SET(CMAKE_BUILD_TYPE "DEBUG")
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "DEBUG")
    #SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

include_directories("src")

# Find Boost
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
#SET(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup iostreams)
find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)

include_directories( ${Boost_INCLUDE_DIRS} )

file(GLOB_RECURSE SOURCES "src/**.cpp")

add_executable(elmapd ${SOURCES} )
target_link_libraries(elmapd ${Boost_LIBRARIES})
target_link_libraries(elmapd OpenMP::OpenMP_CXX)
target_link_libraries(elmapd spdlog::spdlog)
target_compile_definitions(elmapd PRIVATE PYTHON=${PYTHON_FLAG})

add_custom_target(clean_all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/el_mapd
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/*.so
)


set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_BINARY_DIR}/lifelong;
     ${CMAKE_BINARY_DIR}/build;
     ${CMAKE_BINARY_DIR}/*.so"
)



