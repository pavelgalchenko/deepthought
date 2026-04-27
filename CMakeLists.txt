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

# Compile with SPICE -DSPICE=OFF
option(SPICE "Compile with SPICE" ON)

# Compile with Ac Standalone -DSTANDALONE=ON
option(STANDALONE "Compile AcApp as standalone" OFF)

# Compile enabling address sanitizer -DSANITIZER=ON
option(SANITIZER "Enable address sanitizer in debug build" OFF)

# take note of the buid options in Kit/CMakeLists.txt as well
################################################################################
################################################################################
################################################################################

# by default, we will build a Release version. However, feel free to
# include the debug flags by running:
#   $ cmake -DCMAKE_BUILD_TYPE=Debug ..
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(deepthought)
add_executable(Test EXCLUDE_FROM_ALL)

include(CheckCCompilerFlag)
check_c_compiler_flag(-Wno-unused-local-typedefs C_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS)

target_compile_options(deepthought PRIVATE -fpic -Wall -Wshadow -Wno-deprecated)
target_compile_options(deepthought PRIVATE $<$<CONFIG:Release>:-O2>)
target_compile_options(deepthought PRIVATE $<$<CONFIG:Debug>:-O0 -pg -ggdb -fno-omit-frame-pointer>)

# addresssanitizer
target_compile_options(deepthought PRIVATE $<$<BOOL:${SANITIZER}>:-fsanitize=address -fsanitize=undefined>)
target_link_options(deepthought BEFORE PRIVATE $<$<BOOL:${SANITIZER}>:-fsanitize=undefined -fsanitize=address>)

target_compile_options(deepthought PRIVATE $<$<BOOL:${C_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS}>:-Wno-unused-local-typedefs>)

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

target_include_directories(deepthought PRIVATE Kit/Include Include)
target_sources(deepthought PRIVATE ${SOURCE}/42main.c ${42_SOURCES} ${SIM_IPC_SOURCES})

if(SPICE)
    # Based on similar section in GMAT cmake
    set(CSPICE_TEMP_DIR ${CMAKE_SOURCE_DIR}/cspice)

    find_path(CSPICE_DIR
        NAMES include/SpiceUsr.h
        PATHS 
            ${CSPICE_TEMP_DIR}
        DOC "Path to CSpice root directory (containing include/, lib/, etc.)"
        NO_DEFAULT_PATH
        )

    # Make sure CSPICE exists
    if(CSPICE_DIR)
        MESSAGE(STATUS "Found CSPICE: " ${CSPICE_DIR})

        # Set CSPICE library files
        # if(WIN32)
        #     SET(CSPICE_EXT "lib")
        # else()
        SET(CSPICE_EXT "a")
        # endif()
        SET(CSPICE_LIB "${CSPICE_DIR}/lib/cspice.${CSPICE_EXT}")
        # SET(CSPICE_LIB_DEBUG "${CSPICE_DIR}/lib/cspiced.${CSPICE_EXT}")
        SET(CSPICE_INCLUDE_DIR "${CSPICE_DIR}/include")
        if(NOT EXISTS ${CSPICE_LIB_DEBUG})
            SET(CSPICE_LIB_DEBUG ${CSPICE_LIB})
        endif()
    else()
        MESSAGE(ERROR "CSPICE NOT FOUND. Set CSPICE_DIR to path to include/SpiceUsr.h and re-configure.")
    endif()
    add_compile_definitions(_ENABLE_SPICE_)
    
    target_link_libraries(deepthought PUBLIC optimized ${CSPICE_LIB} debug ${CSPICE_LIB_DEBUG})
    target_include_directories(deepthought PUBLIC ${CSPICE_INCLUDE_DIR})
endif()

# Configure Test Target
target_include_directories(Test PRIVATE Kit/Include Include)
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

# configure GUI
target_compile_definitions(deepthought PRIVATE $<$<BOOL:${GUI}>:_ENABLE_GUI_>)