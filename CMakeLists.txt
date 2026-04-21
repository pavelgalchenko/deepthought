# Based shamelessly on CODEX CMakeLists
cmake_minimum_required(VERSION 3.19)

# set project name
project(deepthought VERSION 1.0 LANGUAGES C)

# specify c++ standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")

if (CMAKE_SYSTEM_NAME EQUAL "Windows")
    message(SEND_ERROR "Don't use Windows")
elseif(APPLE)
    set(CMAKE_MACOSX_RPATH 1)
endif()

################################################################################
######################### Configure cmake build options ########################
################################################################################

# Compile with Graphics -DGUI=OFF
option(GUI "Compile with graphics" ON)

# Compile with Shaders -DSHADERS=OFF
option(SHADERS "Compile with shaders" ON)

# Compile with SPICE -DSPICE=OFF
option(SPICE "Compile with SPICE" ON)

# Compile with Ac Standalone -DSTANDALONE=ON
option(STANDALONE "Compile AcApp as standalone" OFF)

# Compile enabling address sanitizer -DSANITIZER=ON
option(SANITIZER "Enable address sanitizer in debug build" OFF)

# take note of the buid options in Kit/CMakeLists.txt as well
################################################################################

# by default, we will build a Release version. However, feel free to
# include the debug flags by running:
#   $ cmake -DCMAKE_BUILD_TYPE=Debug ..
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(deepthought)
add_executable(Test EXCLUDE_FROM_ALL)
link_libraries(m)

include(CheckCCompilerFlag)
check_c_compiler_flag(-Wno-unused-local-typedefs C_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS)

if(SHADERS)
    add_compile_definitions(_USE_SHADERS_)
endif()

set(CMAKE_C_FLAGS "-fpic -Wall -Wshadow -Wno-deprecated " CACHE STRING "Set C Compiler Flags" FORCE)

# explicitly set compile flags for all, debug, and release
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")

if(C_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-local-typedefs")
endif()

# set include directories
include_directories(Include)

# Configure Sources
set(SOURCE Source)

set(SIM_IPC_SOURCES
    PRIVATE
    ${SOURCE}/IPC/SimWriteToFile.c
    ${SOURCE}/IPC/SimWriteToSocket.c
    ${SOURCE}/IPC/SimReadFromFile.c
    ${SOURCE}/IPC/SimReadFromSocket.c
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
    )

target_include_directories(deepthought PRIVATE kit/Include Include)
target_sources(deepthought PRIVATE ${SOURCE}/42main.c ${42_SOURCES} ${SIM_IPC_SOURCES})

if(SPICE)
    add_compile_definitions(_ENABLE_SPICE_)
    set(CSPICE_DIR ${CMAKE_SOURCE_DIR}/cspice)
    find_library(CSPICE_LIB cspice PATHS ${CSPICE_DIR}/lib)
    if (NOT CSPICE_LIB)
        add_library(cspice STATIC IMPORTED)
        set_target_properties(cspice PROPERTIES IMPORTED_LOCATION ${CSPICE_DIR}/lib/cspice.a)
        set_target_properties(cspice PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${CSPICE_DIR}/include)
        target_link_libraries(deepthought PUBLIC cspice)
        target_link_libraries(Test PUBLIC cspice)
    else()
        target_include_directories(deepthought PUBLIC ${CSPICE_DIR}/include)
        target_link_libraries(deepthought PUBLIC ${CSPICE_LIB})

        # #Link spice to Test
        target_include_directories(Test PUBLIC ${CSPICE_DIR}/include)
        target_link_libraries(Test PUBLIC ${CSPICE_LIB})
    endif()
endif()

if(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    target_compile_options(deepthought PUBLIC -O0 -pg -ggdb -fno-omit-frame-pointer)
    if (SANITIZER)
        target_compile_options(deepthought PUBLIC -fsanitize=address -fsanitize=undefined)
        # addresssanitizer linking option if debug
        target_link_options(deepthought BEFORE PUBLIC -fsanitize=undefined -fsanitize=address)
    endif()
endif()

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

# link 42kit
add_subdirectory(Kit)
target_link_libraries(deepthought PRIVATE 42kit)
target_link_libraries(Test PRIVATE 42kit)
# Configure Sources
#   TODO: trying to separate this out, but its here for now for AC_SOURCES and GUI reasons
set(KIT Kit/Source)

set(AC_SOURCES
    PRIVATE
    ${SOURCE}/AcApp.c
    )

if(STANDALONE)
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
    add_executable(AcApp)
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
    # configure GUI
    add_compile_definitions(_ENABLE_GUI_)

    if (GLUT_OR_GLFW STREQUAL "GLUT")
        add_compile_definitions(_USE_GLUT_)
        set(GLUT_SOURCES
            PRIVATE
            ${KIT}/42glutkit.c
            )
        target_sources(deepthought ${GLUT_SOURCES})
        target_sources(Test ${GLUT_SOURCES})
    else()
        add_compile_definitions(_USE_GLFW_)
        find_package(glfw3 REQUIRED)
        target_link_libraries(Test PRIVATE glfw)
        set(GLFW_SOURCES
            PRIVATE
            ${KIT}/42glfwkit.c
            )
        target_sources(deepthought ${GLFW_SOURCES})
        target_sources(Test ${GLFW_SOURCES})
    endif()
endif()