find_package(GTest REQUIRED)
include(GoogleTest)

if(NMPC_STANDALONE)
  function(add_nmpc_ddp_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest nmpc_ddp)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  find_package(ament_cmake_gtest REQUIRED)
  function(add_nmpc_ddp_test NAME)
    ament_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} nmpc_ddp)
  endfunction()
endif()


set(nmpc_ddp_gtest_list
  TestBoxQP
  TestDDPBipedal
  TestDDPVerticalMotion
  TestDDPCentroidalMotion
  )

set(nmpc_ddp_rostest_list
  TestDDPCartPole
  )

foreach(NAME IN LISTS nmpc_ddp_gtest_list)
  add_nmpc_ddp_test(${NAME})
endforeach()

# if(NOT NMPC_STANDALONE)
#   find_package(launch_testing_ament_cmake)
#   foreach(NAME IN LISTS nmpc_ddp_rostest_list)
#     add_nmpc_ddp_test(${NAME} ${NAME})
#     add_launch_test(test/TestDDPCartPole.test.py)
#   endforeach()
# endif()
