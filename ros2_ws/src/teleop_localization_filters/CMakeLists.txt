cmake_minimum_required(VERSION 3.8)
project(teleop_localization_filters)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Header-only; no compiled target, just export the include directory for
# whichever C++ node later depends on this package.
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_kalman_filter_1d_py test/test_kalman_filter_1d.py)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kalman_filter_1d_cpp test/test_kalman_filter_1d.cpp)
  target_include_directories(test_kalman_filter_1d_cpp PRIVATE include)
endif()

ament_package()
