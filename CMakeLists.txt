
cmake_minimum_required(VERSION 3.1...3.20)
project(MyProject)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(Threads REQUIRED)

# find_package(Eigen3 3.1.91 QUIET) #(requires 3.1.91 or greater)

# find_package(TBB QUIET)

find_package(OpenCV 4.0 REQUIRED)
message(STATUS "OpenCV is set to ${OpenCV_INCLUDE_DIRS}")

include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/lib
${PROJECT_SOURCE_DIR}/lib/CDTMAP
${PROJECT_SOURCE_DIR}/lib/CSubdivision
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples)


add_library(CDTMAP STATIC
    lib/CDTMAP/CDTmapcd.cpp
    lib/CDTMAP/CDTcommon.cpp
    lib/CDTMAP/CDTthpp.cpp
    lib/CDTMAP/debug.cpp
)
target_include_directories(CDTMAP PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(CDTMAP PUBLIC ${OpenCV_LIBS})


add_executable(expTCS
test/expTCS.cpp
)
target_link_libraries(expTCS PUBLIC 
${OpenCV_LIBS}
CDTMAP
)

add_executable(expTHPP
test/expTHPP.cpp
)
target_link_libraries(expTHPP PUBLIC 
${OpenCV_LIBS}
CDTMAP
)
add_executable(testTHPP
test/testTHPP.cpp
)
target_link_libraries(testTHPP PUBLIC 
${OpenCV_LIBS}
CDTMAP
)

add_executable(expUTHPP
test/expUTHPP.cpp
)
target_link_libraries(expUTHPP PUBLIC 
${OpenCV_LIBS}
CDTMAP
)
add_executable(testUTHPP
test/testUTHPP.cpp
)
target_link_libraries(testUTHPP PUBLIC 
${OpenCV_LIBS}
CDTMAP
)

add_executable(expTMV
test/expTMV.cpp
)
target_link_libraries(expTMV PUBLIC 
${OpenCV_LIBS}
CDTMAP
)
add_executable(testTMV
test/testTMV.cpp
)
target_link_libraries(testTMV PUBLIC 
${OpenCV_LIBS}
CDTMAP
)

