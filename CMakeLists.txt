cmake_minimum_required(VERSION 3.4.0)

project(MultiBodyVehicle)

set(CMAKE_CXX_STANDARD 11)

# set build env
set(root_dir ${CMAKE_SOURCE_DIR})

if(WIN32)
    set(bullet3_root "C://BULLET_PHYSICS//")
else()
    set(bullet3_root "/opt/bullet3")
endif(WIN32)

include_directories(${root_dir}/message ${root_dir}/inc)

find_package(Bullet CONFIG REQUIRED PATHS ${bullet3_root})

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

if(WIN32)
    add_definitions(-D_USE_MATH_DEFINES)
    add_definitions(-DNOMINMAX)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)


    # add_definitions(-DBT_NO_GLAD)
    message(STATUS "bullet3 find state is ${BULLET_FOUND}")
    message(STATUS "bullet3 include dir is ${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR}")
    message(STATUS "bullet3 lib dir is ${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS}")
    message(STATUS "bullet3 libs is ${BULLET_LIBRARIES}")

    set(vcpkg_root_dir "E:/work/vcpkg/installed/x64-windows")

    include_directories(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR} ${bullet3_root}/include/bullet
        ${bullet3_root}/include/bullet/ThirdPartyLibs/glad
        ${root_dir}/deps ${root_dir}/inc ${vcpkg_root_dir}/include)
    link_directories(${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS} ${root_dir}/deps/lib ${vcpkg_root_dir}/lib)

    # get all bullets libs
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        file(GLOB lib_bullet3 "${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS}/*_Debug.lib")
    else()
        file(GLOB lib_bullet3 "${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS}/*.lib")
    endif()

    message(STATUS "bullet3 libraries:${lib_bullet3}")
    message(STATUS "build type:" ${CMAKE_BUILD_TYPE})

    # set(lib_bullet3 ${BULLET_LIBRARIES})
    set(lib_sys shlwapi)
    set(lib_glog glog.lib gflags.lib)
    set(lib_protobuf libprotobuf.lib)
    set(lib_opengl glad.lib glu32.lib opengl32.lib)
else()
    include_directories(${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR} ${bullet3_root}/include/bullet
        ${bullet3_root}/include/bullet/ThirdPartyLibs/glad)
    link_directories(${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu)

    # get all bullets libs
    set(lib_bullet3 ${BULLET_LIBRARIES} OpenGLWindow gwen)

    set(lib_sys pthread)
    set(lib_glog glog gflags)
    set(lib_protobuf protobuf)
    set(lib_opengl glfw glut) #glad glu32 opengl32
endif(WIN32)

# gui srcs
aux_source_directory(${root_dir}/src/gui gui_srcs)

# common and sim msg lib
aux_source_directory(${root_dir}/message sim_message_srcs)
aux_source_directory(${root_dir}/src/common common_srcs)
add_library(sim_common ${common_srcs} ${sim_message_srcs})
target_link_libraries(sim_common ${lib_glog} ${lib_protobuf} ${lib_sys})

# build hello world
add_executable(hello_world ${root_dir}/src/HelloWorld.cpp)
if(WIN32)
    target_link_libraries(hello_world ${lib_bullet3} ${lib_sys} sim_common ${lib_glog} ${lib_protobuf})
else()
    target_link_libraries(hello_world -Wl,--whole-archive ${lib_bullet3} -Wl,--no-whole-archive BulletRobotics BulletExampleBrowserLib ${lib_sys} sim_common ${lib_glog} ${lib_protobuf})
endif()
target_compile_definitions(hello_world PRIVATE BT_USE_DOUBLE_PRECISION=1)

# build gui hello world
aux_source_directory(${root_dir}/src/basic_example basic_example_srcs)
add_executable(hello_world_gui ${gui_srcs}
    ${basic_example_srcs}
    ${BULLET_ROOT_DIR}/build3/bullet.rc
)
if(WIN32)
    target_link_libraries(hello_world_gui ${lib_bullet3} ${lib_sys} ${lib_opengl} sim_common ${lib_glog} ${lib_protobuf})
else()
    target_link_libraries(hello_world_gui -Wl,--whole-archive ${lib_bullet3} -Wl,--no-whole-archive BulletRobotics BulletExampleBrowserLib ${lib_sys} ${lib_opengl} sim_common ${lib_glog} ${lib_protobuf})   
endif(WIN32)
target_compile_definitions(hello_world_gui PRIVATE BT_USE_DOUBLE_PRECISION=1)

# build chains example
aux_source_directory(${root_dir}/src/chains chains_example_srcs)
add_executable(chains ${gui_srcs}
    ${chains_example_srcs}
    ${BULLET_ROOT_DIR}/build3/bullet.rc
)
if(WIN32)
    target_link_libraries(chains ${lib_bullet3} ${lib_sys} ${lib_opengl} sim_common ${lib_glog} ${lib_protobuf})
else()
    target_link_libraries(chains -Wl,--whole-archive ${lib_bullet3} -Wl,--no-whole-archive BulletRobotics BulletExampleBrowserLib ${lib_sys} ${lib_opengl} sim_common ${lib_glog} ${lib_protobuf})   
endif(WIN32)
target_compile_definitions(chains PRIVATE BT_USE_DOUBLE_PRECISION=1)

# build chains example
aux_source_directory(${root_dir}/src/vehicle_example vehicle_example_srcs)
add_executable(SimpleVehicle ${gui_srcs}
    ${vehicle_example_srcs}
    ${BULLET_ROOT_DIR}/build3/bullet.rc
)
if(WIN32)
    target_link_libraries(SimpleVehicle ${lib_bullet3} ${lib_sys} ${lib_opengl} sim_common ${lib_glog} ${lib_protobuf})
else()
    target_link_libraries(SimpleVehicle -Wl,--whole-archive ${lib_bullet3} -Wl,--no-whole-archive BulletRobotics BulletExampleBrowserLib ${lib_sys} ${lib_opengl} sim_common ${lib_glog} ${lib_protobuf})   
endif(WIN32)
target_compile_definitions(SimpleVehicle PRIVATE BT_USE_DOUBLE_PRECISION=1)

# build parameter writer
aux_source_directory(${root_dir}/src/vehicle_param_writer param_writer_srcs)
add_executable(param_writer ${param_writer_srcs})
if(WIN32)
    target_link_libraries(param_writer sim_common ${lib_glog} ${lib_protobuf} ${lib_bullet3} ${lib_sys} ${lib_opengl})
else()
    target_link_libraries(param_writer -Wl,--whole-archive sim_common ${lib_glog} ${lib_protobuf} ${lib_bullet3} -Wl,--no-whole-archive BulletRobotics BulletExampleBrowserLib ${lib_sys} ${lib_opengl})
endif(WIN32)
target_compile_definitions(param_writer PRIVATE BT_USE_DOUBLE_PRECISION=1)
