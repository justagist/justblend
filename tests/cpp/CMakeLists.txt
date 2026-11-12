find_package(GTest CONFIG)
if(NOT GTest_FOUND)
    find_package(GTest MODULE)
endif()

if(NOT GTest_FOUND)
    message(FATAL_ERROR "GTest not found; install libgtest-dev or set GTest_DIR.")
endif()

# Helper to define a test executable that can include both public and internal headers.
function(jb_add_cpp_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE justblend GTest::gtest GTest::gtest_main)
    target_include_directories(${name} PRIVATE ${PROJECT_SOURCE_DIR}/src)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

jb_add_cpp_test(test_trap test_trap.cpp)
jb_add_cpp_test(test_scurve test_scurve.cpp)
jb_add_cpp_test(test_blends test_blends.cpp)
jb_add_cpp_test(test_corner_handling test_corner_handling.cpp)
jb_add_cpp_test(test_sampling test_sampling.cpp)
jb_add_cpp_test(test_trajectory_features test_trajectory_features.cpp)
jb_add_cpp_test(test_validation test_validation.cpp)
jb_add_cpp_test(test_properties test_properties.cpp)

# Compiles the library sources itself, with Eigen's runtime allocation check
# and the assertions it relies on enabled, so allocations inside the library
# are caught too.
get_target_property(jb_sources justblend SOURCES)
list(TRANSFORM jb_sources PREPEND "${PROJECT_SOURCE_DIR}/")
add_executable(test_realtime test_realtime.cpp ${jb_sources})
target_include_directories(test_realtime PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(test_realtime PRIVATE Eigen3::Eigen GTest::gtest GTest::gtest_main)
target_compile_definitions(test_realtime PRIVATE EIGEN_RUNTIME_NO_MALLOC)
target_compile_options(test_realtime PRIVATE $<IF:$<CXX_COMPILER_ID:MSVC>,/UNDEBUG,-UNDEBUG>)
add_test(NAME test_realtime COMMAND test_realtime)
