/*
 * TWC Manager for ESP32
 *
 * Copyright (C) 2020 Craig Peacock
 * Copyright (C) 2020 Jarl Nicolson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <cstring>
#include <cstdarg>

#include "twc_controller.h"
#include "twc_protocol.h"
#include "functions.h"

namespace esphome {
namespace twc_controller {

static const char *TAG = "twc.protocol";

TeslaController::TeslaController(
    uart::UARTComponent *serial,
    TeslaControllerIO *io,
    uint16_t twcid,
    GPIOPin *flow_control_pin,
    int passive_mode)
    : serial_(serial)
    , flow_control_pin_(flow_control_pin)
    , controller_io_(io)
    , num_connected_chargers_(0)
    , twcid_(twcid)
    , sign_(0x77)
    , max_current_(0)
    , min_current_(0)
    , stopstart_delay_(0)
    , debug_(false)
    , passive_mode_(passive_mode) {
}

void TeslaController::Begin() {
    ESP_LOGD(TAG, "Starting Tesla Controller...");

    // Register callbacks for IO
    controller_io_->onCurrentMessage(
        [this](uint8_t current) { this->SetCurrent(current); });

    receive_index_ = 0;
    if (flow_control_pin_) {
        flow_control_pin_->digital_write(false);
    }
}

// Called in Arduino setup() to start the main controller loop (a FreeRTOS task)
void TeslaController::Startup() {
    ESP_LOGD(TAG, "Starting up Tesla Controller task as primary...");
    xTaskCreate(
        this->startupTask_,
        "TeslaControllerTask",
        2048,
        this,
        1,
        NULL);
}

// FreeRTOS task: sends presence messages then periodic commands
void TeslaController::startupTask_(void *pvParameter) {
    auto *twc = static_cast<TeslaController *>(pvParameter);

    // Wait while in passive mode
    while (twc->passive_mode_) {
        vTaskDelay(pdMS_TO_TICKS(1000 + random(100, 200)));
    }

    // 5 * Presence1
    for (int i = 0; i < 5; ++i) {
        twc->SendPresence();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 5 * Presence2
    for (int i = 0; i < 5; ++i) {
        twc->SendPresence2();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    uint8_t commandNumber = 0;

    for (;;) {
        auto count = twc->ChargersConnected();
        if (count > 0) {
            for (uint8_t i = 0; i < count; ++i) {
                twc->SendHeartbeat(twc->chargers[i]->twcid);
                if (twc->current_changed_) {
                    twc->current_changed_ = false;
                }
                vTaskDelay(pdMS_TO_TICKS(500 + random(50, 100)));

                switch (commandNumber) {
                    case 0: twc->SendCommand(GET_VIN_FIRST, twc->chargers[i]->twcid); break;
                    case 1: twc->SendCommand(GET_VIN_MIDDLE, twc->chargers[i]->twcid); break;
                    case 2: twc->SendCommand(GET_VIN_LAST, twc->chargers[i]->twcid); break;
                    case 3: twc->SendCommand(GET_SERIAL_NUMBER, twc->chargers[i]->twcid); break;
                    case 4: twc->SendCommand(GET_PWR_STATE, twc->chargers[i]->twcid); break;
                    case 5: twc->SendCommand(GET_FIRMWARE_VER_EXT, twc->chargers[i]->twcid); break;
                }

                vTaskDelay(pdMS_TO_TICKS(1000 + random(100, 200)));
            }

            commandNumber = (commandNumber >= 5) ? 0 : commandNumber + 1;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000 + random(100, 200)));
        }
    }

    // Should never reach here
    vTaskDelete(NULL);
}

// Called in main loop() to receive and process incoming data
void TeslaController::Handle() {
    uint8_t receivedChar;

    while (serial_->available()) {
        serial_->read_byte(&receivedChar);

        if (receive_index_ > MAX_PACKET_LENGTH - 1) {
            ESP_LOGE(TAG, "Packet length exceeded");
            receive_index_ = 0;
            return;
        }

        switch (receivedChar) {
            case SLIP_END:
                if (message_started_) {
                    if (receive_index_ <= 2) {
                        // Likely corrupt small packet, reset
                        receive_index_ = 0;
                        break;
                    }
                    ProcessPacket(receive_buffer_, receive_index_);
                    message_started_ = false;
                    receive_index_ = 0;
                } else {
                    message_started_ = true;
                    receive_index_ = 0;
                }
                break;

            case SLIP_ESC: {
                // Block until next byte arrives
                if (serial_->read_array(&receivedChar, 1) != 1) {
                    ESP_LOGE(TAG, "Error while receiving packet data for a packet");
                    return;
                }
                switch (receivedChar) {
                    case SLIP_ESC_END: receive_buffer_[receive_index_++] = SLIP_END; break;
                    case SLIP_ESC_ESC: receive_buffer_[receive_index_++] = SLIP_ESC; break;
                    default: break; // TODO: Error
                }
                break;
            }

            default:
                if (message_started_) {
                    receive_buffer_[receive_index_++] = receivedChar;
                }
                break;
        }
    }
}

void TeslaController::Debug(bool enabled) {
    ESP_LOGD(TAG, "%s Debug", enabled ? "Enabling" : "Disabling");
    debug_ = enabled;
}

void TeslaController::GetSerial(uint16_t secondary_twcid) {
    SendCommand(GET_SERIAL_NUMBER, secondary_twcid);
}

void TeslaController::GetFirmwareVer(uint16_t secondary_twcid) {
    SendCommand(GET_FIRMWARE_VER, secondary_twcid);
}

void TeslaController::GetPowerStatus(uint16_t secondary_twcid) {
    SendCommand(GET_PWR_STATE, secondary_twcid);
}

void TeslaController::GetVin(uint16_t secondary_twcid) {
    SendCommand(GET_VIN_FIRST, secondary_twcid);
    SendCommand(GET_VIN_MIDDLE, secondary_twcid);
    SendCommand(GET_VIN_LAST, secondary_twcid);
}

void TeslaController::SetCurrent(uint8_t current) {
    // set skip flag when current==0
    skip_send_on_zero_current_ = (current == 0);

    if (available_current_ != current) {
        // if SetCurrent(0) was called, force available_current_ to 0 and exit early
        if (current == 0) {
            available_current_ = 0;
            return;
        }
        ESP_LOGD(TAG, "Received current change message, new current %d", current);
        current_changed_ = true;
    }

    // Do handshake and Send 1 heartbeat (as in startup)
    for (int i = 0; i < 5; ++i) {
        SendPresence();
        delay(100);
    }
    for (int i = 0; i < 5; ++i) {
        SendPresence2();
        delay(100);
    }
    for (uint8_t i = 0; i < ChargersConnected(); ++i) {
        SendHeartbeat(chargers[i]->twcid);
        delay(100);
    }

    // Clamp to [min_current_, max_current_]
    available_current_ = clamp(current, min_current_, max_current_);
}

void TeslaController::SendPresence(bool presence2) {
    RESP_PACKET_T presence{};
    PRESENCE_PAYLOAD_T *payload = (PRESENCE_PAYLOAD_T *)&presence.payload;

    presence.command = htons(presence2 ? PRIMARY_PRESENCE2 : PRIMARY_PRESENCE);
    presence.twcid = twcid_;
    payload->sign = sign_;
    payload->max_allowable_current = htons(0x0C80); // TODO: Replace hard-coded value
    for (auto &b : payload->padding) {
        b = 0;
    }
    presence.checksum = CalculateChecksum((uint8_t *)&presence, sizeof(presence));

    SendData((uint8_t *)&presence, sizeof(presence));
}

bool TeslaController::IsCharging() const {
    return total_current_ > 0;
}

void TeslaController::SendPresence2() {
    SendPresence(true);
}

uint8_t TeslaController::ChargersConnected() const {
    return num_connected_chargers_;
}

void TeslaController::SendHeartbeat(uint16_t secondary_twcid) {
    P_HEARTBEAT_T heartbeat{};
    heartbeat.command = htons(PRIMARY_HEARTBEAT);
    heartbeat.src_twcid = twcid_;
    heartbeat.dst_twcid = secondary_twcid;

    if (current_changed_) {
        uint16_t encodedMax = available_current_ * 100;
        heartbeat.state = 0x09; // Limit power
        heartbeat.max_current = htons(encodedMax);
    } else {
        heartbeat.state = 0x00;
        heartbeat.max_current = 0;
    }

    heartbeat.plug_inserted = 0x00;
    for (auto &b : heartbeat.padding) {
        b = 0;
    }
    heartbeat.checksum = CalculateChecksum((uint8_t *)&heartbeat, sizeof(heartbeat));

    SendData((uint8_t *)&heartbeat, sizeof(heartbeat));
}

void TeslaController::SendCommand(uint16_t command, uint16_t send_to) {
    PACKET_T packet{};
    packet.command = htons(command);
    packet.twcid = twcid_;
    packet.secondary_twcid = send_to;
    for (auto &b : packet.payload) {
        b = 0;
    }
    packet.checksum = CalculateChecksum((uint8_t *)&packet, sizeof(packet));

    SendData((uint8_t *)&packet, sizeof(packet));
}

uint8_t TeslaController::CalculateChecksum(uint8_t *buffer, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < length; ++i) {
        checksum += buffer[i];
    }
    return checksum;
}

bool TeslaController::VerifyChecksum(uint8_t *buffer, size_t length) {
    return buffer[length - 1] == CalculateChecksum(buffer, length - 1);
}

void TeslaController::DecodeExtFirmwareVerison(RESP_PACKET_T *firmware_ver) {
    EXT_FIRMWARE_PAYLOAD_T *payload = (EXT_FIRMWARE_PAYLOAD_T *)firmware_ver->payload;
    TeslaConnector *c = GetConnector(firmware_ver->twcid);

    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%d.%d.%d.%d",
        payload->major,
        payload->minor,
        payload->revision,
        payload->extended);

    if (memcmp(&c->firmware_version, payload, 4) != 0) {
        memcpy(&c->firmware_version, payload, 4);
        controller_io_->writeChargerFirmware(firmware_ver->twcid, std::string(buffer));
    }

    if (debug_) {
        ESP_LOGD(TAG, "Decoded: ID: %04x, Firmware Ver: %s", firmware_ver->twcid, buffer);
    }
}

void TeslaController::DecodeSerialNumber(EXTENDED_RESP_PACKET_T *serial) {
    SERIAL_PAYLOAD_T *payload = (SERIAL_PAYLOAD_T *)serial->payload;
    TeslaConnector *c = GetConnector(serial->twcid);

    if (strcmp((char *)c->serial_number, (char *)payload->serial) != 0) {
        strcpy((char *)c->serial_number, (char *)payload->serial);
        controller_io_->writeChargerSerial(serial->twcid, std::string((char *)c->serial_number));
    }

    if (debug_) {
        ESP_LOGD(TAG, "Decoded: ID: %04x, Serial Number: %s", serial->twcid, c->serial_number);
    }
}

void TeslaController::DecodePowerState(EXTENDED_RESP_PACKET_T *power_state) {
    POWERSTATUS_PAYLOAD_T *p = (POWERSTATUS_PAYLOAD_T *)power_state->payload;
    TeslaConnector *c = GetConnector(power_state->twcid);

    if (!c) {
        if (!passive_mode_) return;

        // Accept missed primary presence
        ESP_LOGD(TAG, "New charger seen - adding to controller. ID: %04x, Max Allowable Current: ?", power_state->twcid);

        c = new TeslaConnector(power_state->twcid, 99);
        chargers[num_connected_chargers_++] = c;

        controller_io_->writeCharger(c->twcid, c->max_allowable_current);
        controller_io_->writeTotalConnectedChargers(num_connected_chargers_);
        controller_io_->resetIO(power_state->twcid);
    }

    uint32_t total_kwh = ntohl(p->total_kwh);
    if (total_kwh != c->total_kwh) {
        c->total_kwh = total_kwh;
        controller_io_->writeChargerTotalKwh(power_state->twcid, total_kwh);
    }

    auto updateVoltage = [&](uint16_t &field, uint16_t v, int phase) {
        if (v != field) {
            field = v;
            controller_io_->writeChargerVoltage(power_state->twcid, v, phase);
        }
    };

    updateVoltage(c->phase1_voltage, ntohs(p->phase1_voltage), 1);
    updateVoltage(c->phase2_voltage, ntohs(p->phase2_voltage), 2);
    updateVoltage(c->phase3_voltage, ntohs(p->phase3_voltage), 3);

    auto updateCurrent = [&](uint8_t &field, uint16_t raw, int phase) {
        uint8_t cur = raw / 2;
        if (cur != field) {
            field = cur;
            controller_io_->writeChargerCurrent(power_state->twcid, cur, phase);
            UpdateTotalPhaseCurrent(phase);
        }
    };

    updateCurrent(c->phase1_current, p->phase1_current, 1);
    updateCurrent(c->phase2_current, p->phase2_current, 2);
    updateCurrent(c->phase3_current, p->phase3_current, 3);

    if (debug_) {
        ESP_LOGD(TAG, "Decoded: ID: %04x, Power State...", power_state->twcid);
    }
}

void TeslaController::DecodePrimaryPresence(RESP_PACKET_T *presence, uint8_t num) {
    PRESENCE_PAYLOAD_T *payload = (PRESENCE_PAYLOAD_T *)presence->payload;
    TeslaConnector *c = GetConnector(presence->twcid);

    if (!c) {
        ESP_LOGD(TAG, "New charger seen - adding to controller. ID: %04x, Sign: %02x, Max Allowable Current: %d",
            presence->twcid, payload->sign, ntohs(payload->max_allowable_current));

        uint8_t mac = static_cast<uint8_t>(ntohs(payload->max_allowable_current) / 100);
        c = new TeslaConnector(presence->twcid, mac);
        chargers[num_connected_chargers_++] = c;

        controller_io_->writeCharger(c->twcid, c->max_allowable_current);
        controller_io_->writeTotalConnectedChargers(num_connected_chargers_);
        controller_io_->resetIO(presence->twcid);
    }

    if (debug_) {
        ESP_LOGD(TAG, "Decoded: Primary Presence %d - ID: %02x, Sign: %02x",
            num, presence->twcid, payload->sign);
    }
}

void TeslaController::DecodeSecondaryPresence(RESP_PACKET_T *presence) {
    PRESENCE_PAYLOAD_T *payload = (PRESENCE_PAYLOAD_T *)presence->payload;
    DecodePrimaryPresence(presence, 2); // semantics differ only by payload.command
}

void TeslaController::DecodePrimaryHeartbeat(P_HEARTBEAT_T *heartbeat) {
    if (debug_) {
        ESP_LOGD(TAG, "Decoded: Primary Heartbeat...");
    }
    // TODO: handle reply if needed
}

void TeslaController::DecodeSecondaryHeartbeat(S_HEARTBEAT_T *heartbeat) {
    if (debug_) {
        ESP_LOGD(TAG, "Decoded: Secondary Heartbeat...");
    }
    TeslaConnector *c = GetConnector(heartbeat->src_twcid);

    if (c->state != heartbeat->state) {
        if (heartbeat->state == 4) {
            current_changed_ = true;
        }
        c->state = heartbeat->state;
        UpdateTotalConnectedCars();
        controller_io_->writeChargerState(heartbeat->src_twcid, c->state);
    }

    uint8_t newCurrent = ntohs(heartbeat->actual_current) / 100;
    if (newCurrent != c->GetActualCurrent()) {
        c->SetActualCurrent(newCurrent);
        UpdateTotalActualCurrent();
        controller_io_->writeChargerActualCurrent(heartbeat->src_twcid, newCurrent);
    }
}

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
        case PRIMARY_PRESENCE:   DecodePrimaryPresence((RESP_PACKET_T *)packet, 1); break;
        case PRIMARY_PRESENCE2:  DecodePrimaryPresence((RESP_PACKET_T *)packet, 2); break;
        case SECONDARY_PRESENCE: DecodeSecondaryPresence((RESP_PACKET_T *)packet); break;
        case SECONDARY_HEARTBEAT: DecodeSecondaryHeartbeat((S_HEARTBEAT_T *)packet); break;
        case RESP_VIN_FIRST:
        case RESP_VIN_MIDDLE:
        case RESP_VIN_LAST:       DecodeVin((EXTENDED_RESP_PACKET_T *)packet); break;
        case RESP_PWR_STATUS:    DecodePowerState((EXTENDED_RESP_PACKET_T *)packet); break;
        case RESP_FIRMWARE_VER_EXT: DecodeExtFirmwareVerison((RESP_PACKET__
