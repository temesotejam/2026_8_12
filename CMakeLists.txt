cmake_minimum_required(VERSION 3.16)
project(cores3_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(app_logic lib/app/App.cpp)
target_include_directories(app_logic PUBLIC lib/app)

add_executable(cores3_sim simulator/main.cpp)
target_link_libraries(cores3_sim PRIVATE app_logic)

add_executable(app_tests tests/test_app.cpp)
target_link_libraries(app_tests PRIVATE app_logic)

enable_testing()
add_test(NAME app_logic_tests COMMAND app_tests)
