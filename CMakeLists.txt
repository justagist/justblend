cmake_minimum_required(VERSION 3.18)
project(justblend
    VERSION 0.1.0
    DESCRIPTION "Closed-form N-dimensional trajectory generator with smooth corner blends"
    LANGUAGES CXX)

set(JUSTBLEND_STANDALONE OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(JUSTBLEND_STANDALONE ON)
endif()

option(JUSTBLEND_BUILD_TESTS "Build justblend tests" ${JUSTBLEND_STANDALONE})
option(JUSTBLEND_BUILD_EXAMPLES "Build justblend examples" ${JUSTBLEND_STANDALONE})
option(JUSTBLEND_BUILD_PYTHON_BINDINGS "Build justblend pybind11 module" ${JUSTBLEND_STANDALONE})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(justblend STATIC
    src/trajectory.cpp
    src/trajectory_generator.cpp
    src/trapezoidal.cpp
    src/s_curve.cpp
    src/internal/trap_profile.cpp
    src/internal/scurve_profile.cpp
    src/internal/blend.cpp
    src/internal/planner.cpp
)
add_library(justblend::justblend ALIAS justblend)

target_include_directories(justblend
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(justblend PUBLIC Eigen3::Eigen)
target_compile_features(justblend PUBLIC cxx_std_17)
set_target_properties(justblend PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(MSVC)
    target_compile_options(justblend PRIVATE /W4)
else()
    target_compile_options(justblend PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(JUSTBLEND_BUILD_PYTHON_BINDINGS)
    find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(pybind11 CONFIG REQUIRED)
    pybind11_add_module(_core MODULE bindings/python.cpp)
    target_link_libraries(_core PRIVATE justblend)
    set_target_properties(_core PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/justblend
    )
    install(TARGETS _core DESTINATION justblend)
    install(FILES python/justblend/__init__.py python/justblend/_typing.pyi
            DESTINATION justblend)
endif()

if(JUSTBLEND_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests/cpp)
endif()

if(JUSTBLEND_BUILD_EXAMPLES)
    add_subdirectory(examples/cpp)
endif()

install(TARGETS justblend
    EXPORT justblendTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)
install(DIRECTORY include/justblend DESTINATION include)

# Emit justblendConfig.cmake and justblendTargets.cmake so downstream
# packages can `find_package(justblend REQUIRED)`.
include(CMakePackageConfigHelpers)
install(EXPORT justblendTargets
    FILE justblendTargets.cmake
    NAMESPACE justblend::
    DESTINATION lib/cmake/justblend
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/justblendConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/justblendConfig.cmake"
    INSTALL_DESTINATION lib/cmake/justblend
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/justblendConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/justblendConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/justblendConfigVersion.cmake"
    DESTINATION lib/cmake/justblend
)

# Also emit justblendTargets.cmake into the build tree so downstream
# packages can pick up justblend without a separate install step. Consumers
# set justblend_DIR=<this-build> or add it to CMAKE_PREFIX_PATH.
export(EXPORT justblendTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/justblendTargets.cmake"
    NAMESPACE justblend::
)
