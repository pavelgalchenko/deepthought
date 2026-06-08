
# Compile enabling address sanitizer -DSANITIZER=ON
option(SANITIZER "Enable address sanitizer in debug build" OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Criterion REQUIRED IMPORTED_TARGET criterion)


file(GLOB KIT_TEST_SOURCES "*.c")

foreach(KIT_TEST_SOURCE IN LISTS KIT_TEST_SOURCES)
    get_filename_component(EXE_NAME "${KIT_TEST_SOURCE}" NAME_WE)
    add_executable(${EXE_NAME} "${KIT_TEST_SOURCE}")

    target_link_libraries(${EXE_NAME}
        PRIVATE
            42kit
            PkgConfig::Criterion
    )
    target_compile_options(${EXE_NAME} PRIVATE $<$<CONFIG:Release>:-O2>)
    target_compile_options(${EXE_NAME} PRIVATE $<$<CONFIG:Debug>:-O0 -pg -ggdb -fno-omit-frame-pointer>)
    target_link_options(${EXE_NAME} BEFORE PRIVATE $<$<BOOL:${SANITIZER}>:-fsanitize=address -fsanitize=undefined>)

    # target_compile_definitions(kit_tests PRIVATE _UNIT_TESTING_)

    add_test(NAME ${EXE_NAME} COMMAND ${EXE_NAME})
endforeach()