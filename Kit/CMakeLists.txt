################################################################################
######################### Configure cmake build options ########################
################################################################################

# Compile with Shaders -DSHADERS=OFF
option(SHADERS "Compile with shaders" ON)

# Compile with GSFC Watermark -DGSFC=ON
option(GSFC "Enable GSFC Watermark" OFF)

# Compile enabling address sanitizer -DSANITIZER=ON
option(SANITIZER "Enable address sanitizer in debug build" OFF)

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


# set_target_properties(42kit PROPERTIES VERSION ${PROJECT_VERSION})
# set_target_properties(42kit PROPERTIES PUBLIC_HEADER )

# set include directories
target_include_directories(42kit PUBLIC Include)

add_compile_definitions($<$<BOOL:${REPORT_RESIDUALS}>:_REPORT_RESIDUALS_>)

target_compile_definitions(42kit PRIVATE $<$<BOOL:${GSFC}>:_USE_GSFC_WATERMARK_>)

target_compile_definitions(42kit PRIVATE $<$<BOOL:${DEBUG_DATA}>:_DEBUG_GRAV_ _DEBUG_MAG_>)

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

    if (GLUT_OR_GLFW STREQUAL "GLUT")
        set(GLUT_SOURCES
            PUBLIC
            ${KIT}/42glutkit.c
            )
        target_sources(42kit ${GLUT_SOURCES})
    else()
        target_compile_definitions(42kit PUBLIC _USE_GLFW_)
        find_package(glfw3 REQUIRED)
        target_link_libraries(42kit PRIVATE glfw)
        target_link_libraries(42kit PRIVATE ${GLFW_LIBRARIES})
        set(GLFW_SOURCES
            PUBLIC
            ${KIT}/42glutkit.c
            )
        target_sources(42kit ${GLFW_SOURCES})
    endif()

    set(GL_SOURCES
        PRIVATE
        ${KIT}/glkit.c
        ${KIT}/42glkit.c
        ${KIT}/42gpgpukit.c
        )
    target_sources(42kit ${GL_SOURCES})
endif()
