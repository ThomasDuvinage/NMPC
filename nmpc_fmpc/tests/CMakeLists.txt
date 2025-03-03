find_package(GTest REQUIRED)
include(GoogleTest)

set(nmpc_fmpc_gtest_list
  TestMathUtils
  TestFmpcOscillator
)

set(nmpc_fmpc_rostest_list
  TestFmpcCartPole
)

if(NMPC_STANDALONE)
  function(add_nmpc_fmpc_test NAME)
    add_executable(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest nmpc_fmpc)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  find_package(ament_cmake_gtest REQUIRED)
  function(add_nmpc_fmpc_test NAME)
    ament_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} nmpc_fmpc)
  endfunction()
endif()


foreach(NAME IN LISTS nmpc_fmpc_gtest_list)
  add_nmpc_fmpc_test(${NAME})
endforeach()

# if(NOT NMPC_STANDALONE)
#   find_package(launch_testing_ament_cmake)
#   foreach(NAME IN LISTS nmpc_fmpc_rostest_list)
#     add_nmpc_fmpc_test(${NAME} ${NAME})
#     target_link_libraries(${NAME} nmpc_fmpc)

#     add_launch_test(test/TestFmpcCartPole.test.py)
#     ament_target_dependencies(${NAME}
#       rclcpp
#       visualization_msgs
#       std_srvs
#     )
#   endforeach()
# endif()
