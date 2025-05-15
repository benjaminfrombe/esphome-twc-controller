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

// Constructor
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

// Initialize controller and IO callbacks
void TeslaController::Begin() {
    ESP_LOGD(TAG, "Starting Tesla Controller...");
    controller_io_->onCurrentMessage(
        [this](uint8_t current) { this->SetCurrent(current); });
    receive_index_ = 0;
    if (flow_control_pin_) flow_control_pin_->digital_write(false);
}

// Create FreeRTOS task for controller
void TeslaController::Startup() {
    ESP_LOGD(TAG, "Starting up Tesla Controller task as primary...");
    xTaskCreate(
        this->startupTask_,
        "TeslaControllerTask",
        2048,
        this,
        1,
        NULL
    );
}

// Task: send presence messages then regular commands
void TeslaController::startupTask_(void *pvParameter) {
    auto *twc = static_cast<TeslaController *>(pvParameter);

    // Wait until passive mode is cleared
    while (twc->passive_mode_) {
        vTaskDelay(pdMS_TO_TICKS(1000 + random(100, 200)));
    }

    // Send primary presence 5 times
    for (int i = 0; i < 5; ++i) {
        twc->SendPresence();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Send secondary presence 5 times
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
                if (twc->current_changed_) twc->current_changed_ = false;
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
    // Cleanup if ever reached
    vTaskDelete(NULL);
}

// Main loop: read and decode SLIP packets
void TeslaController::Handle() {
    uint8_t ch;
    while (serial_->available()) {
        serial_->read_byte(&ch);
        if (receive_index_ > MAX_PACKET_LENGTH - 1) {
            ESP_LOGE(TAG, "Packet length exceeded");
            receive_index_ = 0;
            return;
        }
        switch (ch) {
            case SLIP_END:
                if (message_started_) {
                    if (receive_index_ <= 2) {
                        // too small, likely corrupted
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
                if (serial_->read_array(&ch, 1) != 1) {
                    ESP_LOGE(TAG, "Error receiving escaped byte");
                    return;
                }
                if (ch == SLIP_ESC_END) receive_buffer_[receive_index_++] = SLIP_END;
                else if (ch == SLIP_ESC_ESC) receive_buffer_[receive_index_++] = SLIP_ESC;
                break;
            }
            default:
                if (message_started_) receive_buffer_[receive_index_++] = ch;
        }
    }
}

// Enable or disable debug logging
void TeslaController::Debug(bool enabled) {
    ESP_LOGD(TAG, "%s Debug", enabled ? "Enabling" : "Disabling");
    debug_ = enabled;
}

// Convenience getters
void TeslaController::GetSerial(uint16_t id)    { SendCommand(GET_SERIAL_NUMBER,     id); }
void TeslaController::GetFirmwareVer(uint16_t id){ SendCommand(GET_FIRMWARE_VER,      id); }
void TeslaController::GetPowerStatus(uint16_t id){ SendCommand(GET_PWR_STATE,         id); }
void TeslaController::GetVin(uint16_t id)       {
    SendCommand(GET_VIN_FIRST,  id);
    SendCommand(GET_VIN_MIDDLE, id);
    SendCommand(GET_VIN_LAST,   id);
}

// Handle incoming current set messages
void TeslaController::SetCurrent(uint8_t current) {
    // set skip flag when current==0
    skip_send_on_zero_current_ = (current == 0);
    if (available_current_ != current) {
        if (current == 0) {
            available_current_ = 0;
            return;
        }
        ESP_LOGD(TAG, "Received current change message, new current %d", current);
        current_changed_ = true;
    }
    // Handshake presence
    for (int i = 0; i < 5; ++i) { SendPresence();  delay(100); }
    for (int i = 0; i < 5; ++i) { SendPresence2(); delay(100); }
    for (uint8_t i = 0; i < ChargersConnected(); ++i) {
        SendHeartbeat(chargers[i]->twcid);
        delay(100);
    }
    // Clamp within min/max
    available_current_ = clamp(current, min_current_, max_current_);
}

// Build and send primary presence
void TeslaController::SendPresence(bool presence2) {
    RESP_PACKET_T pkt{};
    auto *p = reinterpret_cast<PRESENCE_PAYLOAD_T*>(&pkt.payload);
    pkt.command = htons(presence2 ? PRIMARY_PRESENCE2 : PRIMARY_PRESENCE);
    pkt.twcid = twcid_;
    p->sign = sign_;
    p->max_allowable_current = htons(0x0C80); // TODO: replace hard-coded
    std::memset(p->padding, 0, sizeof(p->padding));
    pkt.checksum = CalculateChecksum(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    SendData(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

// Secondary presence calls primary
void TeslaController::SendPresence2() { SendPresence(true); }

// Build and send heartbeat packet
void TeslaController::SendHeartbeat(uint16_t dst) {
    P_HEARTBEAT_T hb{};
    hb.command = htons(PRIMARY_HEARTBEAT);
    hb.src_twcid = twcid_;
    hb.dst_twcid = dst;
    if (current_changed_) {
        hb.state = 0x09;
        hb.max_current = htons(available_current_ * 100);
    } else {
        hb.state = 0;
        hb.max_current = 0;
    }
    hb.plug_inserted = 0;
    std::memset(hb.padding, 0, sizeof(hb.padding));
    hb.checksum = CalculateChecksum(reinterpret_cast<uint8_t*>(&hb), sizeof(hb));
    SendData(reinterpret_cast<uint8_t*>(&hb), sizeof(hb));
}

// Generic command sender
void TeslaController::SendCommand(uint16_t cmd, uint16_t dst) {
    PACKET_T pkt{};
    pkt.command = htons(cmd);
    pkt.twcid = twcid_;
    pkt.secondary_twcid = dst;
    std::memset(pkt.payload, 0, sizeof(pkt.payload));
    pkt.checksum = CalculateChecksum(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    SendData(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

// Compute simple additive checksum
uint8_t TeslaController::CalculateChecksum(uint8_t *buffer, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 1; i < length; ++i) sum += buffer[i];
    return sum;
}

// Verify trailing checksum byte
bool TeslaController::VerifyChecksum(uint8_t *buffer, size_t length) {
    return buffer[length - 1] == CalculateChecksum(buffer, length - 1);
}

// Process a fully received packet
void TeslaController::ProcessPacket(uint8_t *packet, size_t length) {
    if (debug_) {
        ESP_LOGD(TAG, "Received Packet: ");
        controller_io_->writeRawPacket(packet, length);
    }
    if (!VerifyChecksum(packet, length)) {
        ESP_LOGD(TAG, "Error processing packet - checksum verify failed.");
        return;
    }
    uint16_t cmd = (packet[0] << 8) | packet[1];
    switch (cmd) {
        case PRIMARY_PRESENCE:    DecodePrimaryPresence((RESP_PACKET_T*)packet, 1); break;
        case PRIMARY_PRESENCE2:   DecodePrimaryPresence((RESP_PACKET_T*)packet, 2); break;
        case SECONDARY_PRESENCE:  DecodeSecondaryPresence((RESP_PACKET_T*)packet); break;
        case SECONDARY_HEARTBEAT: DecodeSecondaryHeartbeat((S_HEARTBEAT_T*)packet); break;
        case RESP_VIN_FIRST:
        case RESP_VIN_MIDDLE:
        case RESP_VIN_LAST:       DecodeVin((EXTENDED_RESP_PACKET_T*)packet); break;
        case RESP_PWR_STATUS:     DecodePowerState((EXTENDED_RESP_PACKET_T*)packet); break;
        case RESP_FIRMWARE_VER_EXT: DecodeExtFirmwareVerison((RESP_PACKET_T*)packet); break;
        case RESP_SERIAL_NUMBER:  DecodeSerialNumber((EXTENDED_RESP_PACKET_T*)packet); break;
        case PRIMARY_HEARTBEAT:   DecodePrimaryHeartbeat((P_HEARTBEAT_T*)packet); break;
        default:
            ESP_LOGD(TAG, "Unknown packet type received: 0x%04x", cmd);
            break;
    }
}

// Convert hex string and send
void TeslaController::SendDataFromString(const uint8_t *dataString, size_t length) {
    uint8_t buf[MAX_PACKET_LENGTH];
    uint8_t size = hexCharacterStringToBytes(buf, dataString, length);
    SendData(buf, size);
}

// SLIP-encode and transmit packet
void TeslaController::SendData(uint8_t *packet, size_t length) {
    if (skip_send_on_zero_current_) {
        ESP_LOGD(TAG, "Skip SendData: last SetCurrent was 0");
        return;
    }
    if (length > MAX_PACKET_LENGTH) {
        ESP_LOGD(TAG, "Error - packet larger than maximum allowable size!");
        return;
    }
    uint16_t cmd = (packet[0] << 8) | packet[1];
    if (cmd == WRITE_ID_DATE || cmd == WRITE_MODEL_NO) {
        ESP_LOGD(TAG, "WARNING! WRITE COMMANDS ATTEMPTED! THESE CAN PERMANENTLY BREAK YOUR TWC. COMMANDS BLOCKED!");
        return;
    }

    uint8_t out[MAX_PACKET_LENGTH];
    size_t j = 0;
    out[j++] = SLIP_END;
    for (size_t i = 0; i < length; ++i) {
        if (packet[i] == SLIP_END)      { out[j++] = SLIP_ESC; out[j++] = SLIP_ESC_END; }
        else if (packet[i] == SLIP_ESC) { out[j++] = SLIP_ESC; out[j++] = SLIP_ESC_ESC; }
        else out[j++] = packet[i];
    }
    out[j++] = SLIP_END;
    out[j++] = 0xFF;

    if (debug_) {
        ESP_LOGV(TAG, "Sent packet: ");
        for (size_t i = 0; i < j; ++i) ESP_LOGD(TAG, "%02x", out[i]);
    }
    if (flow_control_pin_) flow_control_pin_->digital_write(true);
    serial_->write_array(out, j);
    serial_->flush();
    if (flow_control_pin_) flow_control_pin_->digital_write(false);
}

// Setters for current limits
void TeslaController::SetMaxCurrent(uint8_t max_current) { ESP_LOGD(TAG, "Setting maximum current to %d", max_current); max_current_ = max_current; }
void TeslaController::SetMinCurrent(uint8_t min_current) { ESP_LOGD(TAG, "Setting minimum current to %d", min_current); min_current_ = min_current; }

// Helpers for tracking connected cars and currents
void TeslaController::UpdateTotalConnectedCars() {
    uint8_t connected = 0;
    for (uint8_t i = 0; i < num_connected_chargers_; ++i) if (chargers[i]->state != 0) ++connected;
    controller_io_->writeTotalConnectedCars(connected);
}

void TeslaController::UpdateTotalActualCurrent() {
    total_current_ = 0;
    for (uint8_t i = 0; i < num_connected_chargers_; ++i) total_current_ += chargers[i]->GetActualCurrent();
    if (debug_) ESP_LOGD(TAG, "Updating actual current to %u", total_current_);
    controller_io_->writeActualCurrent(total_current_);
}

void TeslaController::UpdateTotalPhaseCurrent(uint8_t phase) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < num_connected_chargers_; ++i) sum += chargers[i]->GetPhaseCurrent(phase);
    controller_io_->writeChargerTotalPhaseCurrent(sum, phase);
}

TeslaConnector* TeslaController::GetConnector(uint16_t twcid) {
    for (uint8_t i = 0; i < num_connected_chargers_; ++i) {
        if (chargers[i]->twcid == twcid) return chargers[i];
    }
    return nullptr;
}

// Decode VIN over multiple packets
void TeslaController::DecodeVin(EXTENDED_RESP_PACKET_T *vin_data) {
    VIN_PAYLOAD_T *vp = (VIN_PAYLOAD_T*)vin_data->payload;
    TeslaConnector *c = GetConnector(vin_data->twcid);
    uint8_t *vin = c->GetVin();
    bool changed = false;
    switch (ntohs(vin_data->command)) {
        case RESP_VIN_FIRST:
            if (std::memcmp(vin, vp->vin, sizeof(vp->vin))) { changed = true; std::memcpy(vin, vp->vin, sizeof(vp->vin)); }
            break;
        case RESP_VIN_MIDDLE:
            if (std::memcmp(vin+7, vp->vin, sizeof(vp->vin))) { changed = true; std::memcpy(vin+7, vp->vin, sizeof(vp->vin)); }
            break;
        case RESP_VIN_LAST:
            if (std::memcmp(vin+14, vp->vin, 3)) { changed = true; std::memcpy(vin+14, vp->vin, 3); }
            break;
    }
    if (changed) {
        size_t len = strlen((char*)vin);
        if (len == 17) controller_io_->writeChargerConnectedVin(vin_data->twcid, (char*)vin);
        else if (len == 0) controller_io_->writeChargerConnectedVin(vin_data->twcid, "0");
    }
    if (debug_) {
        if (strlen((char*)vp->vin) == 0) ESP_LOGD(TAG, "No Car Connected");
        else ESP_LOGD(TAG, "VIN: %s", vp->vin);
    }
}

// Decode power state and notify IO
void TeslaController::DecodePowerState(EXTENDED_RESP_PACKET_T *power_state) {
    POWERSTATUS_PAYLOAD_T *p = (POWERSTATUS_PAYLOAD_T*)power_state->payload;
    TeslaConnector *c = GetConnector(power_state->twcid);
    if (!c) {
        if (!passive_mode_) return;
        ESP_LOGD(TAG, "New charger seen - adding to controller. ID: %04x", power_state->twcid);
        c = new TeslaConnector(power_state->twcid, 99);
        chargers[num_connected_chargers_++] = c;
        controller_io_->writeCharger(c->twcid, c->max_allowable_current);
        controller_io_->writeTotalConnectedChargers(num_connected_chargers_);
        controller_io_->resetIO(power_state->twcid);
    }
    uint32_t kwh = ntohl(p->total_kwh);
    if (kwh != c->total_kwh) { c->total_kwh = kwh; controller_io_->writeChargerTotalKwh(c->twcid, kwh); }
    auto vol = [&](uint16_t raw, uint16_t &field, int ph){ uint16_t v=ntohs(raw); if(v!=field){field=v;controller_io_->writeChargerVoltage(c->twcid,v,ph);} };
    vol(p->phase1_voltage, c->phase1_voltage, 1);
    vol(p->phase2_voltage, c->phase2_voltage, 2);
    vol(p->phase3_voltage, c->phase3_voltage, 3);
    auto cur = [&](uint16_t raw, uint8_t &field, int ph){ uint8_t v=raw/2; if(v!=field){field=v;controller_io_->writeChargerCurrent(c->twcid,v,ph);UpdateTotalPhaseCurrent(ph);} };
    cur(p->phase1_current, c->phase1_current, 1);
    cur(p->phase2_current, c->phase2_current, 2);
    cur(p->phase3_current, c->phase3_current, 3);
    if (debug_) ESP_LOGD(TAG,"Decoded: ID: %04x, Power State", c->twcid);
}

// Decode primary presence
void TeslaController::DecodePrimaryPresence(RESP_PACKET_T *presence, uint8_t num) {
    PRESENCE_PAYLOAD_T *p = (PRESENCE_PAYLOAD_T*)presence->payload;
    TeslaConnector *c = GetConnector(presence->twcid);
    if (!c) {
        ESP_LOGD(TAG, "New charger seen - adding to controller. ID: %04x, Sign: %02x, Max Curr: %d",
            presence->twcid, p->sign, ntohs(p->max_allowable_current));
        uint8_t mac = ntohs(p->max_allowable_current)/100;
        c = new TeslaConnector(presence->twcid, mac);
        chargers[num_connected_chargers_++] = c;
        controller_io_->writeCharger(c->twcid, c->max_allowable_current);
        controller_io_->writeTotalConnectedChargers(num_connected_chargers_);
        controller_io_->resetIO(presence->twcid);
    }
    if (debug_) ESP_LOGD(TAG, "Decoded: Primary Presence %d - ID: %02x, Sign: %02x", num, presence->twcid, p->sign);
}

// Decode secondary presence (same as primary for now)
void TeslaController::DecodeSecondaryPresence(RESP_PACKET_T *presence) {
    DecodePrimaryPresence(presence, 2);
}

// Decode heartbeats
void TeslaController::DecodePrimaryHeartbeat(P_HEARTBEAT_T *hb) {
    if (debug_) ESP_LOGD(TAG, "Decoded: Primary Heartbeat - ID: %02x, To %02x", hb->src_twcid, hb->dst_twcid);
}

void TeslaController::DecodeSecondaryHeartbeat(S_HEARTBEAT_T *hb) {
    if (debug_) ESP_LOGD(TAG, "Decoded: Secondary Heartbeat - ID: %02x, Status: %02x", hb->src_twcid, hb->state);
    TeslaConnector *c = GetConnector(hb->src_twcid);
    if (c->state != hb->state) {
        if (hb->state == 4) current_changed_ = true;
        c->state = hb->state;
        UpdateTotalConnectedCars();
        controller_io_->writeChargerState(c->twcid, c->state);
    }
    uint8_t actual = ntohs(hb->actual_current)/100;
    if (actual != c->GetActualCurrent()) {
        c->SetActualCurrent(actual);
        UpdateTotalActualCurrent();
        controller_io_->writeChargerActualCurrent(c->twcid, actual);
    }
}

} // namespace twc_controller
} // namespace esphome
