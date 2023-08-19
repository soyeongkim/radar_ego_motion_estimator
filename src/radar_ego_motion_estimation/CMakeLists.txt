cmake_minimum_required(VERSION 3.2)
project(radar_ego_motion_estimation)

add_compile_options(-std=c++14)

set(CATKIN_PACKAGE_DEPENDENCIES

    # open3d_catkin
)

# find_package(Open3D HINTS /home/chuorok/open3d_install/lib/cmake)

# if (NOT Open3D_FOUND)
# message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
# endif ()

# find_package(OpenMP QUIET)
# if (OpenMP_FOUND)
# add_compile_options("${OpenMP_CXX_FLAGS}")
# add_definitions(-Dopen3d_slam_ros_OPENMP_FOUND=${OpenMP_FOUND})
# endif()

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

# open 3d required
# include(FetchContent)

# FetchContent_Declare(fmt
# GIT_REPOSITORY https://github.com/fmtlib/fmt.git
# GIT_TAG master
# )
# FetchContent_MakeAvailable(fmt)

# include(ExternalProject)
# ExternalProject_Add(
# external_open3d
# PREFIX open3d
# GIT_REPOSITORY https://github.com/intel-isl/Open3D.git
# GIT_TAG master
# GIT_SHALLOW ON
# UPDATE_COMMAND ""
# # Check out https://github.com/intel-isl/Open3D/blob/master/CMakeLists.txt
# # For the full list of available options.
# CMAKE_ARGS
# -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
# -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
# -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
# -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
# -DGLIBCXX_USE_CXX11_ABI=${GLIBCXX_USE_CXX11_ABI}
# -DSTATIC_WINDOWS_RUNTIME=${STATIC_WINDOWS_RUNTIME}
# -DBUILD_SHARED_LIBS=ON
# -DBUILD_PYTHON_MODULE=OFF
# -DBUILD_EXAMPLES=OFF
# )
find_package(catkin REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    # velodyne_pcl
)

# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
    CATKIN_DEPENDS
)

include_directories(
    include
    include/ini_handler_cpp
    ${catkin_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# link_directories(${Open3D_LIBRARY_DIRS})
# include_directories(${Open3D_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/radar_ego_velocity_estimator.cpp
    src/radar_point_cloud.cpp
)

# ExternalProject_Get_Property(external_open3d INSTALL_DIR)
add_dependencies(${PROJECT_NAME}

    # external_open3d
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")

target_link_libraries(${PROJECT_NAME}

    # INTERFACE Open3D
    PUBLIC ${LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}

    # fmt::fmt
    # Threads::Threads
    # ${Open3D_INCLUDE_DIRS}
    ${OpenMP_CXX_LIBRARIES}
)

# target_include_directories(${PROJECT_NAME} PUBLIC ${Open3D_INCLUDE_DIRS})

# # target_compile_features(Open3DHelper INTERFACE cxx_std_14)
# target_compile_definitions(${PROJECT_NAME} INTERFACE _GLIBCXX_USE_CXX11_ABI=$<BOOL:${GLIBCXX_USE_CXX11_ABI}>)
# target_include_directories(${PROJECT_NAME} INTERFACE "${INSTALL_DIR}/include" "${INSTALL_DIR}/include/open3d/3rdparty")
# target_link_directories(${PROJECT_NAME} INTERFACE "${INSTALL_DIR}/lib")
