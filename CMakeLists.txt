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

# Phase 8 deliberately compiles the same boat_protocol codec/ABI that is used
# by the production communication and control firmware.
add_library(boat_protocol_host
  lib/boat_protocol/boat_protocol.cpp
)
target_include_directories(boat_protocol_host PUBLIC lib/boat_protocol)

add_library(production_link
  lib/virtual_link/ProductionUartLink.cpp
  lib/virtual_link/ProductionProtocolSystem.cpp
)
target_include_directories(production_link PUBLIC lib/virtual_link)
target_link_libraries(production_link PUBLIC virtual_link boat_protocol_host)

# Phase 9 imports the production controller command-ingress and replay-window
# sources, then runs them against the same production wire codec from Phase 8.
add_library(production_control_host
  lib/production_control/production_control.cpp
  lib/production_control/command_replay.cpp
  lib/production_control/command_ingress.cpp
)
target_include_directories(production_control_host PUBLIC
  lib/production_control
  lib/boat_protocol
)
target_link_libraries(production_control_host PUBLIC boat_protocol_host)

add_library(production_command_link
  lib/virtual_link/ProductionCommandTelemetrySystem.cpp
)
target_include_directories(production_command_link PUBLIC lib/virtual_link)
target_link_libraries(production_command_link PUBLIC
  production_link
  production_control_host
)

add_executable(cores3_sim simulator/main.cpp)
target_link_libraries(cores3_sim PRIVATE app_logic virtual_hw)

add_executable(dual_controller_sim simulator/dual_controller_main.cpp)
target_link_libraries(dual_controller_sim PRIVATE virtual_link)

add_executable(production_protocol_sim simulator/production_protocol_main.cpp)
target_link_libraries(production_protocol_sim PRIVATE production_link)

add_executable(production_command_telemetry_sim
  simulator/production_command_telemetry_main.cpp
)
target_link_libraries(production_command_telemetry_sim PRIVATE
  production_command_link
)

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

add_executable(phase8_tests tests/test_phase8.cpp)
target_link_libraries(phase8_tests PRIVATE production_link)

add_executable(phase9_tests tests/test_phase9.cpp)
target_link_libraries(phase9_tests PRIVATE production_command_link)

enable_testing()
add_test(NAME app_logic_tests COMMAND app_tests)
add_test(NAME virtual_hardware_tests COMMAND virtual_hw_tests)
add_test(NAME integration_tests COMMAND integration_tests)
add_test(NAME phase3_fault_tests COMMAND phase3_tests)
add_test(NAME phase4_bno08x_tests COMMAND phase4_tests)
add_test(NAME phase5_ina226_tests COMMAND phase5_tests)
add_test(NAME phase6_vl53l5cx_tests COMMAND phase6_tests)
add_test(NAME phase7_dual_uart_tests COMMAND phase7_tests)
add_test(NAME phase8_production_protocol_tests COMMAND phase8_tests)
add_test(NAME phase9_production_command_telemetry_tests COMMAND phase9_tests)
