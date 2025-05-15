#include <cstring>
#include <cstdarg>

#include "twc_controller.h"
#include "twc_protocol.h"
#include "functions.h"

namespace esphome {
namespace twc_controller {

static const char *TAG = "twc.protocol";

// ... [alles bovenaan blijft ongewijzigd, je originele code blijft geldig tot aan het einde van DecodeSecondaryHeartbeat()] ...

void TeslaController::ProcessPacket(uint8_t *packet, size_t length) {
    if (debug_) {
        ESP_LOGD(TAG, "Received Packet:");
        controller_io_->writeRawPacket(packet, length);
    }

    if (!VerifyChecksum(packet, length)) {
        ESP_LOGD(TAG, "Error processing packet - checksum verify failed.");
        return;
    }

    uint16_t command = (packet[0] << 8) | packet[1];
    switch (command) {
        case PRIMARY_PRESENCE:
            DecodePrimaryPresence((RESP_PACKET_T *)packet, 1);
            break;
        case PRIMARY_PRESENCE2:
            DecodePrimaryPresence((RESP_PACKET_T *)packet, 2);
            break;
        case SECONDARY_PRESENCE:
            DecodeSecondaryPresence((RESP_PACKET_T *)packet);
            break;
        case SECONDARY_HEARTBEAT:
            DecodeSecondaryHeartbeat((S_HEARTBEAT_T *)packet);
            break;
        case RESP_VIN_FIRST:
        case RESP_VIN_MIDDLE:
        case RESP_VIN_LAST:
            DecodeVin((EXTENDED_RESP_PACKET_T *)packet);
            break;
        case RESP_PWR_STATUS:
            DecodePowerState((EXTENDED_RESP_PACKET_T *)packet);
            break;
        case RESP_FIRMWARE_VER_EXT:
            DecodeExtFirmwareVerison((RESP_PACKET_T *)packet);
            break;
        default:
            if (debug_) {
                ESP_LOGW(TAG, "Unknown packet command: 0x%04X", command);
            }
            break;
    }
}

}  // namespace twc_controller
}  // namespace esphome
