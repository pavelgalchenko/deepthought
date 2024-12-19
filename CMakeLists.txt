# Based shamelessly on CODEX CMakeLists
cmake_minimum_required(VERSION 3.16)

# set project name
project(deepthought VERSION 1.0 LANGUAGES C)

# specify c++ standard
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")

if (CMAKE_SYSTEM_NAME EQUAL "Windows")
    message(SEND_ERROR "Don't use Windows")
elseif(APPLE)
    set(CMAKE_MACOSX_RPATH 1)
endif()

add_executable(deepthought)


# # Cross compiling: cmake -DCROSS=ON ..
# option(CROSS "Cross Compile for Target System" OFF)
# if(CROSS)
#     #TODO for CD tasks
# else()
#     # Taken shamelessly from CODEX
#     # cmake installs libraries to /usr/local/lib by default. Let's set the
#     # RPATH to that to make sure the program actually runs!!
#     # Note: Default install rpath is set to "". Was able to make this work
#     # as long as we run "sudo ldconfig" after installing the libraries.
#     set(CMAKE_INSTALL_RPATH "$ORIGIN")
#     set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
# endif()


include(CheckCCompilerFlag)
check_c_compiler_flag(-Wno-unused-local-typedefs C_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS)

# Compile with Graphics -DGUI=OFF
option(GUI "Compile with graphics" ON)

# Compile with Shaders -DSHADERS=OFF
option (SHADERS "Compile with shaders" ON )
if(SHADERS)
    add_compile_definitions(_USE_SHADERS_)
endif()

# Compile with SPICE -DSPICE=OFF
option(SPICE "Compile with SPICE" ON)
if(SPICE)
    add_compile_definitions(_ENABLE_SPICE_)
    set(CSPICE_DIR ${CMAKE_SOURCE_DIR}/cspice)
    find_library(CSPICE_LIB cspice NAMES cspice PATHS ${CSPICE_DIR}/lib)
    target_include_directories(deepthought PUBLIC ${CSPICE_DIR}/include)
    target_link_libraries(deepthought PUBLIC ${CSPICE_LIB})
    # target_link_directories(deepthought PRIVATE ${CSPICE_DIR}/lib)
endif()


set(CMAKE_C_FLAGS "-fpic -Wall -Wshadow -Wno-deprecated " CACHE STRING "Set C Compiler Flags" FORCE)

# by default, we will build a Release version. However, feel free to
# include the debug flags by running:
#   $ cmake -DCMAKE_BUILD_TYPE=Debug ..
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# turning on the BUILD_SHARED_LIBS flag causes the add_library() to create .so files
# option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# explicitly set compile flags for all, debug, and release
set(CMAKE_C_FLAGS_DEBUG "-pg")
set(CMAKE_C_FLAGS_RELEASE "-O2")


if(C_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-local-typedefs")
endif()

# set include directories
include_directories(
    Include
    Kit/Include
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFYAML REQUIRED libfyaml>=0.9)
include_directories(${LIBFYAML_INCLUDE_DIRS})
link_directories(${LIBFYAML_LIBRARY_DIRS})


set(KIT Kit/Source)
set(SOURCE Source)

set(SIM_IPC_SOURCES
    PRIVATE
    ${SOURCE}/IPC/SimWriteToFile.c
    ${SOURCE}/IPC/SimWriteToSocket.c
    ${SOURCE}/IPC/SimReadFromFile.c
    ${SOURCE}/IPC/SimReadFromSocket.c
    )

set(KIT_SOURCES
    ${KIT}/iokit.c
    ${KIT}/dcmkit.c
    ${KIT}/envkit.c
    ${KIT}/fswkit.c
    ${KIT}/geomkit.c
    ${KIT}/mathkit.c
    ${KIT}/nrlmsise00kit.c
    ${KIT}/orbkit.c
    ${KIT}/radbeltkit.c
    ${KIT}/sigkit.c
    ${KIT}/sphkit.c
    ${KIT}/timekit.c
    ${KIT}/docoptkit.c
    )

    set(42_SOURCES
    PRIVATE
    ${SOURCE}/42main.c
    ${SOURCE}/42exec.c
    ${SOURCE}/42actuators.c
    ${SOURCE}/42cmd.c
    ${SOURCE}/42dynamics.c
    ${SOURCE}/42environs.c
    ${SOURCE}/42ephem.c
    ${SOURCE}/42fsw.c
    ${SOURCE}/42init.c
    ${SOURCE}/42ipc.c
    ${SOURCE}/42jitter.c
    ${SOURCE}/42joints.c
    ${SOURCE}/42optics.c
    ${SOURCE}/42perturb.c
    ${SOURCE}/42report.c
    ${SOURCE}/42sensors.c
    ${SOURCE}/42nos3.c
    ${SOURCE}/42dsm.c
    ${KIT}/dsmkit.c
    ${KIT}/navkit.c
    )

set(AC_KIT_SOURCES
    PRIVATE
    ${KIT}/dcmkit
    ${KIT}/mathkit
    ${KIT}/fswkit
    ${KIT}/iokit
    ${KIT}/timekit
    )

set(AC_IPC_SOURCES
    PRIVATE
    ${SOURCE}/IPC/AppReadFromFile
    ${SOURCE}/IPC/AppWriteToSocket
    ${SOURCE}/IPC/AppReadFromSocket
    ${SOURCE}/IPC/AppWriteToFile
    )

set(AC_SOURCES
    ${SOURCE}/AcApp.c
    )

add_library(42kit SHARED ${KIT_SOURCES})

target_sources(deepthought ${42_SOURCES} ${SIM_IPC_SOURCES} ${AC_SOURCES})
target_include_directories(deepthought PUBLIC kit/Include PRIVATE Include)
target_link_libraries(deepthought PRIVATE 42kit)

# TODO: split this up more
# configure code into libraries
target_include_directories(deepthought PUBLIC ${LIBFYAML_INCLUDE_DIRS})
target_link_libraries(deepthought PUBLIC  ${LIBFYAML_LIBRARIES})
target_compile_options(deepthought PUBLIC ${LIBFYAML_CFLAGS})
target_link_options(deepthought PUBLIC ${LIBFYAML_LDFLAGS})
target_link_libraries(deepthought PUBLIC ${CMAKE_DL_LIBS})

target_include_directories(42kit PUBLIC ${LIBFYAML_INCLUDE_DIRS})
target_compile_options(42kit PUBLIC ${LIBFYAML_CFLAGS})
target_link_options(42kit PUBLIC ${LIBFYAML_LDFLAGS})

if (GUI)
    # For graphics interface, choose GLUT or GLFW GUI libraries
    # GLUT is well known, but GLFW is better for newer Mac's hires displays
    # -DGLUT_OR_GLFW=GLFW
    set(GLUT_OR_GLFW GLUT CACHE STRING "Use GLUT or GLFW for graphics")
    set_property(CACHE GLUT_OR_GLFW PROPERTY STRINGS GLUT GLFW)
    find_package(OpenGL REQUIRED)
    add_compile_definitions(_ENABLE_GUI_)
    add_compile_definitions(GL_SILENCE_DEPRECATION)

    if (GLUT_OR_GLFW STREQUAL "GLUT")
        add_compile_definitions(_USE_GLUT_)
        set(GUI_SOURCES
            PUBLIC
            Include/42constants.h
            Include/42.h
            Kit/Source/iokit.c
            Kit/Source/glkit.c
            Kit/Source/mathkit.c

            PRIVATE
            Include/42gl.h
            Include/42glut.h
            Source/42gl.c
            Source/42glut.c
            Source/42gpgpu.c
            )
        target_sources(deepthought ${GUI_SOURCES})
        # link_libraries(gui PUBLIC  kit)
        find_package(GLUT REQUIRED)
        target_include_directories(deepthought PRIVATE ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
        target_link_libraries(deepthought PUBLIC ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${CMAKE_DL_LIBS})
        target_link_options(deepthought PRIVATE ${OPENGL_LDFLAGS} ${GLUT_LDFLAGS})
        target_compile_options(deepthought PRIVATE ${OPENGL_CFLAGS} ${GLUT_CFLAGS})
    else()
        # add_compile_definitions(_USE_GLFW_)
        # add_library(gui
        #     Source/42gl.c
        #     Kit/Source/glkit.c
        #     Source/42gpgpu.c
        #     Source/42glfw.c
        # )
        # find_package(GLFW REQUIRED)
        # include_directories(${GLFW_INCLUDE_DIRS})
        # target_link_libraries(gui ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})
    endif()
    # target_link_libraries(deepthought PUBLIC gui)
endif()
