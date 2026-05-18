
# Compile enabling address sanitizer -DSANITIZER=ON
option(SANITIZER "Enable address sanitizer in debug build" OFF)


find_package(PkgConfig REQUIRED)
pkg_check_modules(Criterion REQUIRED IMPORTED_TARGET criterion)

add_executable(kit_tests 
    jdkit_tests.c
    rationalkit_tests.c
    )

target_link_libraries(kit_tests
    PRIVATE
        42kit
        PkgConfig::Criterion
)
target_compile_options(kit_tests PRIVATE $<$<CONFIG:Release>:-O2>)
target_compile_options(kit_tests PRIVATE $<$<CONFIG:Debug>:-O0 -pg -ggdb -fno-omit-frame-pointer>)
target_link_options(kit_tests BEFORE PRIVATE $<$<BOOL:${SANITIZER}>:-fsanitize=address -fsanitize=undefined>)

target_compile_definitions(kit_tests PRIVATE _UNIT_TESTING_)

add_test(NAME kit_tests COMMAND kit_tests)