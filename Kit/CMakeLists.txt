################################################################################
######################### Configure cmake build options ########################
################################################################################

# Compile with Shaders -DSHADERS=OFF
option(SHADERS "Compile with shaders" ON)

# Compile with GSFC Watermark -DGSFC=ON
option(GSFC "Enable GSFC Watermark" OFF)

# Compile with SPICE -DSPICE=OFF
option(SPICE "Compile with SPICE" ON)

# Compile enabling address sanitizer -DSANITIZER=ON
option(SANITIZER "Enable address sanitizer in debug build" OFF)

# Compile to output debug information -DDEBUG_DATA=ON
option(DEBUG_DATA "Enable outputing model debug data" OFF)

# Compile enabling residual reporting -DREPORT_RESIUALS=ON
option(REPORT_RESIDUALS "Enable navigation measurement residual reporting" OFF)

# Ignore long not being smaller than long long -DIGNORE_LONG=TRUE
option(IGNORE_LONG "Enable navigation measurement residual reporting" FALSE)

# For graphics interface, choose GLUT or GLFW GUI libraries
# GLUT is well known, but GLFW is better for newer Mac's hires displays
# -DGLUT_OR_GLFW=GLFW
set(GLUT_OR_GLFW GLUT CACHE STRING "Use GLUT or GLFW for graphics")
set_property(CACHE GLUT_OR_GLFW PROPERTY STRINGS GLUT GLFW)

################################################################################


# Configure Sources
set(KIT Source)

set(KIT_SOURCES
    ${KIT}/iokit.c
    ${KIT}/dcmkit.c
    ${KIT}/envkit.c
    ${KIT}/fswkit.c
    ${KIT}/geomkit.c
    ${KIT}/rationalkit.c
    ${KIT}/mathkit.c
    ${KIT}/nrlmsise00kit.c
    ${KIT}/orbkit.c
    ${KIT}/radbeltkit.c
    ${KIT}/sigkit.c
    ${KIT}/sphkit.c
    ${KIT}/jdkit.c
    ${KIT}/timekit.c
    ${KIT}/docoptkit.c
    ${KIT}/dsmkit.c
    ${KIT}/navkit.c
    )

add_library(42kit SHARED ${KIT_SOURCES})

target_link_libraries(42kit PUBLIC m)

target_compile_options(42kit PRIVATE -fpic -Wall -Wshadow -Wno-deprecated)
target_compile_options(42kit PRIVATE $<$<CONFIG:Release>:-O2>)
target_compile_options(42kit PRIVATE $<$<CONFIG:Debug>:-fno-omit-frame-pointer>)

# addresssanitizer
target_compile_options(42kit PRIVATE $<$<BOOL:${SANITIZER}>:-fsanitize=address -fsanitize=undefined>)
target_link_options(42kit BEFORE PRIVATE $<$<BOOL:${SANITIZER}>:-fsanitize=undefined -fsanitize=address>)

target_compile_options(42kit PRIVATE $<$<BOOL:${C_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS}>:-Wno-unused-local-typedefs>)

target_compile_definitions(42kit PRIVATE $<$<BOOL:${SHADERS}>:_USE_SHADERS_>)

target_compile_definitions(42kit PRIVATE $<$<BOOL:${IGNORE_LONG}>:_IGNORE_LONG_>)


# set_target_properties(42kit PROPERTIES VERSION ${PROJECT_VERSION})
# set_target_properties(42kit PROPERTIES PUBLIC_HEADER )

# set include directories
target_include_directories(42kit PUBLIC Include)

add_compile_definitions($<$<BOOL:${REPORT_RESIDUALS}>:_REPORT_RESIDUALS_>)

target_compile_definitions(42kit PRIVATE $<$<BOOL:${GSFC}>:_USE_GSFC_WATERMARK_>)

target_compile_definitions(42kit PRIVATE $<$<BOOL:${DEBUG_DATA}>:_DEBUG_GRAV_ _DEBUG_MAG_>)


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
    
    target_link_libraries(42kit PUBLIC optimized ${CSPICE_LIB} debug ${CSPICE_LIB_DEBUG})
    target_include_directories(42kit PUBLIC ${CSPICE_INCLUDE_DIR})
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

if (GUI)
    # configure GUI
    find_package(OpenGL REQUIRED)
    add_compile_definitions(GL_SILENCE_DEPRECATION)
    target_link_libraries(42kit PUBLIC ${OPENGL_LIBRARIES})

    find_package(GLUT REQUIRED)
    target_link_libraries(42kit PUBLIC ${GLUT_LIBRARIES})

    set(GL_SOURCES
        PRIVATE
        ${KIT}/glkit.c
        ${KIT}/42glkit.c
        ${KIT}/42gpgpukit.c
        )
    target_sources(42kit ${GL_SOURCES})

    if (GLUT_OR_GLFW STREQUAL "GLUT")
        set(GLUT_SOURCES
            PRIVATE
            ${KIT}/42glutkit.c
            )
        target_sources(42kit ${GLUT_SOURCES})
    else()
        target_compile_definitions(42kit PUBLIC _USE_GLFW_)
        find_package(glfw3 REQUIRED)
        target_link_libraries(42kit INTERFACE glfw)
        target_link_libraries(42kit PRIVATE ${GLFW_LIBRARIES})
        set(GLFW_SOURCES
            PUBLIC
            ${KIT}/42glfwkit.c
            )
        target_sources(42kit ${GLFW_SOURCES})
    endif()

endif()
