cmake_minimum_required(VERSION 3.16)
project(cores3_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(app_logic lib/app/App.cpp)
target_include_directories(app_logic PUBLIC lib/app)

add_library(virtual_hw
  lib/virtual_hw/VirtualHardware.cpp
  lib/virtual_hw/VirtualBno08x.cpp
  lib/virtual_hw/VirtualIna226.cpp
  lib/virtual_hw/VirtualVl53l5cx.cpp
)
target_include_directories(virtual_hw PUBLIC lib/virtual_hw)
target_link_libraries(virtual_hw PUBLIC app_logic)

add_library(virtual_link
  lib/virtual_link/VirtualDuplexUart.cpp
  lib/virtual_link/DualControllerSystem.cpp
)
target_include_directories(virtual_link PUBLIC lib/virtual_link)

add_executable(cores3_sim simulator/main.cpp)
target_link_libraries(cores3_sim PRIVATE app_logic virtual_hw)

add_executable(dual_controller_sim simulator/dual_controller_main.cpp)
target_link_libraries(dual_controller_sim PRIVATE virtual_link)

add_executable(app_tests tests/test_app.cpp)
target_link_libraries(app_tests PRIVATE app_logic)

add_executable(virtual_hw_tests tests/test_virtual_hw.cpp)
target_link_libraries(virtual_hw_tests PRIVATE virtual_hw)

add_executable(integration_tests tests/test_integration.cpp)
target_link_libraries(integration_tests PRIVATE app_logic virtual_hw)

add_executable(phase3_tests tests/test_phase3.cpp)
target_link_libraries(phase3_tests PRIVATE app_logic virtual_hw)

add_executable(phase4_tests tests/test_phase4.cpp)
target_link_libraries(phase4_tests PRIVATE app_logic virtual_hw)

add_executable(phase5_tests tests/test_phase5.cpp)
target_link_libraries(phase5_tests PRIVATE app_logic virtual_hw)

add_executable(phase6_tests tests/test_phase6.cpp)
target_link_libraries(phase6_tests PRIVATE app_logic virtual_hw)

add_executable(phase7_tests tests/test_phase7.cpp)
target_link_libraries(phase7_tests PRIVATE virtual_link)

enable_testing()
add_test(NAME app_logic_tests COMMAND app_tests)
add_test(NAME virtual_hardware_tests COMMAND virtual_hw_tests)
add_test(NAME integration_tests COMMAND integration_tests)
add_test(NAME phase3_fault_tests COMMAND phase3_tests)
add_test(NAME phase4_bno08x_tests COMMAND phase4_tests)
add_test(NAME phase5_ina226_tests COMMAND phase5_tests)
add_test(NAME phase6_vl53l5cx_tests COMMAND phase6_tests)
add_test(NAME phase7_dual_uart_tests COMMAND phase7_tests)
