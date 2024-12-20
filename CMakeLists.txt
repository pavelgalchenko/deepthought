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
add_executable(Test EXCLUDE_FROM_ALL)

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
    target_link_directories(deepthought PUBLIC ${CSPICE_DIR}/lib)

    #Link spice to Test
    target_include_directories(Test PUBLIC ${CSPICE_DIR}/include)
    target_link_libraries(Test PUBLIC ${CSPICE_LIB})
    target_link_directories(Test PUBLIC ${CSPICE_DIR}/lib)
endif()


set(CMAKE_C_FLAGS "-fpic -Wall -Wshadow -Wno-deprecated " CACHE STRING "Set C Compiler Flags" FORCE)

# by default, we will build a Release version. However, feel free to
# include the debug flags by running:
#   $ cmake -DCMAKE_BUILD_TYPE=Debug ..
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


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

# Configure Sources
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
    ${KIT}/dcmkit.c
    ${KIT}/mathkit.c
    ${KIT}/fswkit.c
    ${KIT}/iokit.c
    ${KIT}/timekit.c
    )

set(AC_IPC_SOURCES
    PRIVATE
    ${SOURCE}/IPC/AppReadFromFile.c
    ${SOURCE}/IPC/AppWriteToSocket.c
    ${SOURCE}/IPC/AppReadFromSocket.c
    ${SOURCE}/IPC/AppWriteToFile.c
    )

set(AC_SOURCES
    PRIVATE
    ${SOURCE}/AcApp.c
    )

add_library(42kit SHARED ${KIT_SOURCES})
set_target_properties(42kit PROPERTIES LIBRARY_OUTPUT_DIRECTORY Kit)

target_include_directories(deepthought PRIVATE kit/Include Include)
target_sources(deepthought PRIVATE ${SOURCE}/42main.c ${42_SOURCES} ${SIM_IPC_SOURCES})

# Compile with Slosh -DCFDSLOSH=ON
option(CFDSLOSH "Enable CFD slosh" OFF)
if(CFDSLOSH)
    target_sources(deepthought PRIVATE ${SOURCE}/42CfdSlosh.c)
    add_compile_definitions(_ENABLE_CFD_SLOSH_)
endif()

# Compile with FFTB -DFFTB=ON
option(FFTB "Enable FFTB" OFF)
if(FFTB)
    target_sources(deepthought PRIVATE ${SOURCE}/42fftb.c)
    add_compile_definitions(_ENABLE_FFTB_CODE_)
endif()

# Compile with GSFC Watermark -DGSFC=ON
option(GSFC "Enable GSFC Watermark" OFF)
if(GSFC)
    add_compile_definitions(_USE_GSFC_WATERMARK_)
endif()

# Compile to output debug information -DDEBUG_DATA=ON
option(DEBUG_DATA "Enable outputing model debug data" OFF)
if(DEBUG_DATA)
    add_compile_definitions(_DEBUG_GRAV_)
    add_compile_definitions(_DEBUG_MAG_)
endif()

# Configure 42kit with libfyaml publicly to share with targets
# if something in deepthought is using yaml, it is also using iokit
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFYAML REQUIRED libfyaml>=0.9)
target_include_directories(42kit PUBLIC kit/Include)
target_include_directories(42kit PUBLIC ${LIBFYAML_INCLUDE_DIRS})
target_link_directories(42kit PUBLIC ${LIBFYAML_LIBRARY_DIRS})
target_link_libraries(42kit PUBLIC ${LIBFYAML_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(deepthought PRIVATE 42kit)

# Configure Test Target
target_include_directories(Test PRIVATE kit/Include Include)
target_sources(Test
    PRIVATE
    Tests/tests.c
    Tests/mathkit_tests.c
    # Tests/navkit_tests.c
    Tests/test_lib.c
    ${42_SOURCES}
    ${SIM_IPC_SOURCES}
    )
target_link_libraries(Test PRIVATE 42kit)

# Compile with Ac Standalone -DSTANDALONE=ON
option(STANDALONE "Compile AcApp and DeepThought as standalone" OFF)
if(STANDALONE)
    add_executable(AcApp EXCLUDE_FROM_ALL)
    target_compile_definitions(AcApp PRIVATE _AC_STANDALONE_)
    target_sources(AcApp ${AC_SOURCES} ${AC_KIT_SOURCES} ${AC_IPC_SOURCES})
    target_include_directories(AcApp PRIVATE kit/Include)
    target_include_directories(AcApp PRIVATE ${LIBFYAML_INCLUDE_DIRS})
    target_link_directories(AcApp PRIVATE ${LIBFYAML_LIBRARY_DIRS})
    target_link_libraries(AcApp PRIVATE ${LIBFYAML_LIBRARIES} ${CMAKE_DL_LIBS})

    target_compile_definitions(deepthought PRIVATE _AC_STANDALONE_)
else()
    target_sources(deepthought ${AC_SOURCES})
    target_sources(Test ${AC_SOURCES})
endif()

if (GUI)
    # For graphics interface, choose GLUT or GLFW GUI libraries
    # GLUT is well known, but GLFW is better for newer Mac's hires displays
    # -DGLUT_OR_GLFW=GLFW
    set(GLUT_OR_GLFW GLUT CACHE STRING "Use GLUT or GLFW for graphics")
    set_property(CACHE GLUT_OR_GLFW PROPERTY STRINGS GLUT GLFW)
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    add_compile_definitions(_ENABLE_GUI_)
    add_compile_definitions(GL_SILENCE_DEPRECATION)
    target_link_libraries(deepthought PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
    target_link_libraries(Test PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

    set(GL_SOURCES
        PRIVATE
        ${KIT}/glkit.c
        Source/42gl.c
        Source/42gpgpu.c
        )
    target_sources(deepthought ${GL_SOURCES})
    target_sources(Test ${GL_SOURCES})

    if (GLUT_OR_GLFW STREQUAL "GLUT")
        add_compile_definitions(_USE_GLUT_)
        set(GLUT_SOURCES
            PRIVATE
            Source/42glut.c
            )
        target_sources(deepthought ${GLUT_SOURCES})
        target_sources(Test ${GLUT_SOURCES})
    else()
        # TODO: doesn't work
        add_compile_definitions(_USE_GLFW_)
        set(GLFW_SOURCES
            PRIVATE
            Source/42glfw.c
            )
        target_sources(deepthought ${GLFW_SOURCES})
        find_package(glfw3 REQUIRED)
        target_link_libraries(deepthought PRIVATE ${GLFW_LIBRARIES})
        target_link_libraries(Test PRIVATE ${GLFW_LIBRARIES})
    endif()
endif()
