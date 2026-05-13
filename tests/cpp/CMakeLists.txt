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
    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
    )
    add_test(NAME ${name} COMMAND ${name})
endfunction()

jb_add_cpp_test(test_trap test_trap.cpp)
jb_add_cpp_test(test_scurve test_scurve.cpp)
jb_add_cpp_test(test_blends test_blends.cpp)
jb_add_cpp_test(test_corner_handling test_corner_handling.cpp)
jb_add_cpp_test(test_sampling test_sampling.cpp)
