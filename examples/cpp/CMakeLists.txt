function(jb_add_example name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE justblend)
endfunction()

jb_add_example(example_basic_2d 01_basic_2d.cpp)
jb_add_example(example_robot_joints_scurve 02_robot_joints_scurve.cpp)
jb_add_example(example_blend_comparison 03_blend_comparison.cpp)
