cmake_minimum_required(VERSION 3.19)

project(lib42kit VERSION 1.0 LANGUAGES C)

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

if(DEFINED $ENV{CONDA_PREFIX})
    list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
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

# Compile with GSFC Watermark -DGSFC=ON
option(GSFC "Enable GSFC Watermark" OFF)

# Compile to output debug information -DDEBUG_DATA=ON
option(DEBUG_DATA "Enable outputing model debug data" OFF)

# Compile enabling residual reporting -DREPORT_RESIUALS=ON
option(REPORT_RESIDUALS "Enable navigation measurement residual reporting" OFF)

# For graphics interface, choose GLUT or GLFW GUI libraries
# GLUT is well known, but GLFW is better for newer Mac's hires displays
# -DGLUT_OR_GLFW=GLFW
set(GLUT_OR_GLFW GLUT CACHE STRING "Use GLUT or GLFW for graphics")
set_property(CACHE GLUT_OR_GLFW PROPERTY STRINGS GLUT GLFW)

################################################################################

# by default, we will build a Release version. However, feel free to
# include the debug flags by running:
#   $ cmake -DCMAKE_BUILD_TYPE=Debug ..
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

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

# Configure Sources
set(KIT Source)

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
    ${KIT}/dsmkit.c
    ${KIT}/navkit.c
    )

add_library(42kit SHARED ${KIT_SOURCES})
# set_target_properties(42kit PROPERTIES VERSION ${PROJECT_VERSION})
# set_target_properties(42kit PROPERTIES PUBLIC_HEADER )

# set include directories
target_include_directories(42kit PUBLIC Include)
# target_include_directories(42kit PRIVATE Source)

if(REPORT_RESIUDALS)
    add_compile_definitions(REPORT_RESIDUALS)
endif()

if(SPICE)
    add_compile_definitions(_ENABLE_SPICE_)
    if (TARGET cspice)
        target_link_libraries(42kit PUBLIC cspice)
    else()
        set(CSPICE_DIR ../cspice)
        find_library(CSPICE_LIB cspice PATHS ${CSPICE_DIR}/lib)
        if (NOT CSPICE_LIB)
            set(CSPICE_DIR ../cspice)
            find_library(CSPICE_LIB cspice PATHS ${CSPICE_DIR}/lib)
            add_library(cspice STATIC IMPORTED)
            set_target_properties(cspice PROPERTIES IMPORTED_LOCATION ${CSPICE_DIR}/lib/cspice.a)
            set_target_properties(cspice PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${CSPICE_DIR}/include)
            target_link_libraries(42kit PUBLIC cspice)
        else()
            target_include_directories(42kit PUBLIC ${CSPICE_DIR}/include)
            target_link_libraries(42kit PUBLIC ${CSPICE_LIB})
        endif()
    endif()
endif()

if(GSFC)
    add_compile_definitions(_USE_GSFC_WATERMARK_)
endif()

if(DEBUG_DATA)
    add_compile_definitions(_DEBUG_GRAV_)
    add_compile_definitions(_DEBUG_MAG_)
endif()

# Configure 42kit with libfyaml publicly to share with targets
# if something in deepthought is using yaml, it is also using iokit
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFYAML REQUIRED libfyaml>=0.8)
if (LIBFYAML_VERSION VERSION_GREATER_EQUAL "0.9.6")
    target_compile_definitions(42kit PUBLIC __USE_LIBFYAML_CORE__)
endif()

target_include_directories(42kit PUBLIC kit/Include)
target_include_directories(42kit PUBLIC ${LIBFYAML_INCLUDE_DIRS})
target_link_directories(42kit PUBLIC ${LIBFYAML_LIBRARY_DIRS})
target_link_libraries(42kit PUBLIC ${LIBFYAML_LIBRARIES} ${CMAKE_DL_LIBS} ${BLAS_LIBRARIES})
target_link_libraries(42kit PRIVATE ${LAPACK_LIBRARIES})

if(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    target_compile_options(42kit PUBLIC -O0 -pg -ggdb -fno-omit-frame-pointer)
    
    if (SANITIZER)
        target_compile_options(42kit PUBLIC -fsanitize=address -fsanitize=undefined)
        # addresssanitizer linking option if debug
        target_link_options(42kit BEFORE PUBLIC -fsanitize=undefined -fsanitize=address)
    endif()
endif()


if (GUI)
    # configure GUI
    find_package(OpenGL REQUIRED)
    add_compile_definitions(_ENABLE_GUI_)
    add_compile_definitions(GL_SILENCE_DEPRECATION)
    target_link_libraries(42kit PUBLIC ${OPENGL_LIBRARIES})

    find_package(GLUT REQUIRED)
    target_link_libraries(42kit PUBLIC ${GLUT_LIBRARIES})

    if (GLUT_OR_GLFW STREQUAL "GLFW")
        add_compile_definitions(_USE_GLFW_)
        find_package(glfw3 REQUIRED)
        target_link_libraries(42kit PRIVATE glfw)
        target_link_libraries(42kit PUBLIC ${GLFW_LIBRARIES})
    endif()

    set(GL_SOURCES
        PRIVATE
        ${KIT}/glkit.c
        ${KIT}/42glkit.c
        ${KIT}/42gpgpukit.c
        )
    target_sources(42kit ${GL_SOURCES})
endif()
