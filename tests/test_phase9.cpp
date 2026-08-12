#include "ProductionCommandTelemetrySystem.h"
#include "boat_protocol.h"
#include "production_control.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace cores3sim;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void clearOneShot(ProductionCommandTelemetryInput& input) {
  input.action = ProductionAction::None;
  input.comm_to_control_fault = InjectedUartFault::None;
  input.control_to_comm_fault = InjectedUartFault::None;
  input.corrupt_canonical_crc = false;
}

void run(ProductionCommandTelemetrySystem& system,
         ProductionCommandTelemetryInput& input,
         std::uint32_t from, std::uint32_t to) {
  for (std::uint32_t t = from; t <= to; t += 10) {
    system.step(t, input);
    clearOneShot(input);
  }
}
}

int main() {
  static_assert(sizeof(boat::ControlOutputPayload) == 72,
                "production ControlOutput ABI");
  static_assert(sizeof(boat::ControlSnapshotPayload) == 190,
                "production ControlSnapshot ABI");
  static_assert(sizeof(boat::InaStatusPayload) == 32,
                "production INA ABI");
  static_assert(sizeof(boat::VescTelemetryPayload) == 48,
                "production VESC ABI");
  static_assert(sizeof(boat::ActuatorStatePayload) == 36,
                "production actuator ABI");
  static_assert(sizeof(boat::SystemHealthPayload) == 36,
                "production health ABI");

  {
    ProductionCommandTelemetrySystem system;
    ProductionCommandTelemetryInput input;
    run(system, input, 0, 250);
    auto status = system.step(250, input);
    require(status.comm_link_fresh && status.host_heartbeat_fresh,
            "production bidirectional link must become fresh");
    require(status.control_output_rx >= 1 && status.control_snapshot_rx >= 1 &&
                status.ina_status_rx >= 1 && status.vesc_telemetry_rx >= 1 &&
                status.actuator_state_rx >= 1 && status.system_health_rx >= 1,
            "COMM must decode the full six-frame production telemetry burst");
    require(std::fabs(status.latest_ina_bus_v - 12.2f) < 0.01f &&
                std::fabs(status.latest_ina_current_a - 2.0f) < 0.01f &&
                std::fabs(status.latest_vesc_erpm - 1200.0f) < 0.1f &&
                status.latest_actuator_pca_ready == 1,
            "representative production telemetry payload values must decode");

    // Begin a real ControlModeCommand but drop only its first ACK. The COMM
    // side must retry after 100 ms with the same inner request/command IDs.
    input.action = ProductionAction::SetMode;
    input.mode = boat::ControlAutoWaypoint;
    system.step(300, input);
    clearOneShot(input);

    input.control_to_comm_fault = InjectedUartFault::Drop;
    system.step(310, input);
    clearOneShot(input);
    run(system, input, 320, 440);
    status = system.step(450, input);
    require(status.pending_state == PendingCommandState::Accepted &&
                !status.pending_active,
            "dropped mode ACK must recover through pending retry");
    require(status.pending_attempts == 2,
            "mode command must need exactly one retry");
    require(status.control_mode == production_control::ControlMode::AutoWaypoint,
            "production command ingress must apply mode once");
    require(status.ingress_applied == 1 && status.ingress_duplicates >= 1 &&
                status.duplicate_acks_rx >= 1,
            "replayed mode command must return Duplicate without reapply");
    require(system.stats().pending_retries >= 1,
            "COMM pending retry counter");

    // The production WaypointSet path has its own ACK and duplicate/revision
    // behavior outside CommandIngress. Drop the first WaypointAck as well.
    input.action = ProductionAction::SetWaypoint;
    input.waypoint_latitude_deg = 36.40100;
    input.waypoint_longitude_deg = 136.60050;
    system.step(500, input);
    clearOneShot(input);
    input.control_to_comm_fault = InjectedUartFault::Drop;
    system.step(510, input);
    clearOneShot(input);
    run(system, input, 520, 650);
    status = system.step(660, input);
    require(status.pending_state == PendingCommandState::Accepted &&
                status.waypoint_count == 1 && status.waypoint_revision == 1,
            "waypoint retry must finish and preserve accepted waypoint");
    require(status.last_waypoint_ack_status == 2 &&
                status.last_waypoint_ack_reason == 7,
            "retried waypoint must use production Duplicate/Revision ACK");
    require(status.duplicate_acks_rx >= 2,
            "mode and waypoint duplicate ACKs both counted");

    // The production preflight can now ARM/START because heartbeat, GNSS and
    // waypoint state are all valid and AutoWaypoint does not require manual refresh.
    input.action = ProductionAction::Arm;
    system.step(700, input);
    clearOneShot(input);
    run(system, input, 710, 760);
    status = system.step(760, input);
    require(status.safety == production_control::AuthoritativeSafety::ArmedIdle,
            "real ARM CommandPayload must pass production-style preflight");

    input.action = ProductionAction::Start;
    system.step(800, input);
    clearOneShot(input);
    run(system, input, 810, 1450);
    status = system.step(1450, input);
    require(status.safety == production_control::AuthoritativeSafety::Running &&
                status.running && !status.failsafe_stop,
            "configured AutoWaypoint system must remain RUNNING");
    require(status.control_output_rx >= 10 && status.control_snapshot_rx >= 10 &&
                status.ina_status_rx >= 10 && status.vesc_telemetry_rx >= 10 &&
                status.actuator_state_rx >= 10 && status.system_health_rx >= 10,
            "full production telemetry bundle must continue at 10 Hz class");
    require(status.latest_health_mode == boat::ControlAutoWaypoint &&
                status.latest_health_safety == 3,
            "SystemHealth must expose production mode and RUNNING safety");

    input.action = ProductionAction::Stop;
    system.step(1500, input);
    clearOneShot(input);
    run(system, input, 1510, 1580);
    status = system.step(1580, input);
    require(status.safety == production_control::AuthoritativeSafety::Disarmed &&
                !status.running &&
                status.stop_reason == production_control::StopReason::Stop,
            "production STOP must return the configured system to DISARMED");
    require(status.safety_command_acks_rx >= 3,
            "ARM START STOP must all generate CommandAck frames");
  }

  {
    // Production syntax failures are deliberately not inserted into the replay
    // window. A malformed canonical CRC therefore gets a rejection ACK with no
    // replay entry; the COMM pending command keeps retrying until its 1200 ms
    // timeout. This test preserves that current production behavior exactly.
    ProductionCommandTelemetrySystem system;
    ProductionCommandTelemetryInput input;
    run(system, input, 0, 250);
    input.action = ProductionAction::SetManual;
    input.corrupt_canonical_crc = true;
    system.step(300, input);
    clearOneShot(input);
    run(system, input, 310, 1520);
    const auto status = system.step(1520, input);
    require(status.pending_state == PendingCommandState::Timeout &&
                !status.pending_active,
            "malformed command must eventually hit COMM command timeout");
    require(status.ingress_malformed >= 1,
            "production ingress must classify bad canonical CRC as malformed");
    require(system.stats().pending_timeouts == 1 &&
                system.stats().pending_retries >= 1,
            "timeout/retry counters must reflect malformed pending command");
  }

  {
    // Full production telemetry is also the feedback used to gate COMM
    // heartbeat. Losing CONTROL->COMM must eventually stop COMM heartbeat and
    // then stop an active CONTROL node through the 500 ms host timeout.
    ProductionCommandTelemetrySystem system;
    ProductionCommandTelemetryInput input;
    run(system, input, 0, 250);
    input.action = ProductionAction::SetMode;
    input.mode = boat::ControlAutoWaypoint;
    system.step(300, input); clearOneShot(input);
    run(system, input, 310, 380);
    input.action = ProductionAction::SetWaypoint;
    input.waypoint_latitude_deg = 36.40100;
    system.step(400, input); clearOneShot(input);
    run(system, input, 410, 480);
    input.action = ProductionAction::Arm;
    system.step(500, input); clearOneShot(input);
    run(system, input, 510, 580);
    input.action = ProductionAction::Start;
    system.step(600, input); clearOneShot(input);
    run(system, input, 610, 700);
    auto status = system.step(700, input);
    require(status.running, "failsafe test must start RUNNING");

    input.control_to_comm_connected = false;
    run(system, input, 710, 2300);
    status = system.step(2300, input);
    require(!status.comm_link_fresh && status.failsafe_stop && !status.running &&
                status.stop_reason == production_control::StopReason::Heartbeat,
            "full telemetry loss must propagate through gated heartbeat to failsafe");
    require(system.stats().failsafe_stops == 1,
            "full-telemetry failsafe must latch once");
  }

  std::cout << "All phase-9 production command/telemetry tests passed.\n";
  return 0;
}
