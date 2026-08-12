cmake_minimum_required(VERSION 3.16)
project(cores3_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(app_logic lib/app/App.cpp)
target_include_directories(app_logic PUBLIC lib/app)

add_library(virtual_hw lib/virtual_hw/VirtualHardware.cpp)
target_include_directories(virtual_hw PUBLIC lib/virtual_hw)
target_link_libraries(virtual_hw PUBLIC app_logic)

add_executable(cores3_sim simulator/main.cpp)
target_link_libraries(cores3_sim PRIVATE app_logic virtual_hw)

add_executable(app_tests tests/test_app.cpp)
target_link_libraries(app_tests PRIVATE app_logic)

add_executable(virtual_hw_tests tests/test_virtual_hw.cpp)
target_link_libraries(virtual_hw_tests PRIVATE virtual_hw)

add_executable(integration_tests tests/test_integration.cpp)
target_link_libraries(integration_tests PRIVATE app_logic virtual_hw)

add_executable(phase3_tests tests/test_phase3.cpp)
target_link_libraries(phase3_tests PRIVATE app_logic virtual_hw)

enable_testing()
add_test(NAME app_logic_tests COMMAND app_tests)
add_test(NAME virtual_hardware_tests COMMAND virtual_hw_tests)
add_test(NAME integration_tests COMMAND integration_tests)
add_test(NAME phase3_fault_tests COMMAND phase3_tests)
