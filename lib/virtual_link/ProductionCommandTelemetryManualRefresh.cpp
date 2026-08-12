#include "ProductionCommandTelemetrySystem.h"

#include <cstddef>

namespace cores3sim {

void ProductionCommandTelemetrySystem::sendManualRefresh(std::uint32_t now_ms) {
  if (!manual_refresh_enabled_ || !manual_refresh_mask_ || pending_.active ||
      production_control::modeUsesWaypoint(controller_.mode()) ||
      now_ms - last_manual_refresh_ms_ < kManualRefreshMs) {
    return;
  }

  boat::ManualCommandPayload command{};
  command.protocolVersion = boat::kVersion;
  command.reserved[0] = manual_refresh_mask_;
  command.requestId = request_id_next_++;
  command.commandSequence = command_sequence_next_++;
  command.sourceUs = static_cast<std::uint64_t>(now_ms) * 1000ULL;
  command.leftFrontWing = manual_refresh_left_;
  command.rightFrontWing = manual_refresh_right_;
  command.rearYaw = manual_refresh_rear_;
  command.propulsion = manual_refresh_propulsion_;
  command.canonicalCrc = boat::canonicalCrc(
      &command, offsetof(boat::ManualCommandPayload, canonicalCrc));

  if (sendCommFrame(now_ms, boat::Type::ManualCommand,
                    &command, sizeof(command))) {
    last_manual_refresh_ms_ = now_ms;
    ++stats_.manual_refreshes_sent;
  }
}

ProductionCommandTelemetryStatus
ProductionCommandTelemetrySystem::stepWithManualRefresh(
    std::uint32_t now_ms,
    const ProductionCommandTelemetryInput& input) {
  // The production communication application remembers the requested manual
  // values, then emits fresh ManualCommand IDs every 200 ms while the selected
  // operation is a non-waypoint mode. Keep that stream separate from the
  // PendingCommand retry mechanism, which resends the *same* inner IDs.
  if (input.action == ProductionAction::SetManual) {
    manual_refresh_mask_ = input.manual_mask;
    manual_refresh_left_ = input.manual_left;
    manual_refresh_right_ = input.manual_right;
    manual_refresh_rear_ = input.manual_rear;
    manual_refresh_propulsion_ = input.manual_propulsion;
  }

  if (input.action == ProductionAction::Stop ||
      input.action == ProductionAction::Disarm ||
      input.action == ProductionAction::Estop) {
    manual_refresh_enabled_ = false;
    // Prevent the persistent "last pending command was accepted" status from
    // re-enabling the old manual stream on a later tick. A new SetManual action
    // repopulates the mask and can enable a fresh stream after its ACK arrives.
    manual_refresh_mask_ = 0;
  }

  ProductionCommandTelemetryStatus result = step(now_ms, input);

  if (manual_refresh_mask_ &&
      status_.pending_type == static_cast<std::uint8_t>(boat::Type::ManualCommand) &&
      status_.pending_state == PendingCommandState::Accepted &&
      !status_.pending_active) {
    if (!manual_refresh_enabled_) {
      manual_refresh_enabled_ = true;
      last_manual_refresh_ms_ = now_ms;
    }
  }

  // A failsafe/FAULT is not an operating state where the COMM side should
  // continue refreshing actuator requests.
  if (status_.failsafe_stop ||
      status_.safety == production_control::AuthoritativeSafety::Fault ||
      status_.safety == production_control::AuthoritativeSafety::EStop) {
    manual_refresh_enabled_ = false;
    manual_refresh_mask_ = 0;
  }

  sendManualRefresh(now_ms);
  result = status_;
  return result;
}

}  // namespace cores3sim
