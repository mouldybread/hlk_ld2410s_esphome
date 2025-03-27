#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"  // Changed from uart_device.h
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/button/button.h"
#include <vector>
#include <map>  // Added for std::map

namespace esphome {
namespace hlk_ld2410s {

static const char *const TAG = "hlk_ld2410s";
static const uint8_t MAX_GATES = 9;

// Configuration frame constants
static const uint8_t CONFIG_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CONFIG_FRAME_END[] = {0x04, 0x03, 0x02, 0x01};
static const uint16_t CONFIG_FRAME_MIN_LENGTH = 8;  // Header(4) + Command(2) + Length(2)
static const uint32_t ACK_TIMEOUT_MS = 1000;

enum class CommandWord : uint16_t {
    ENABLE_CONFIGURATION = 0xFF01,
    DISABLE_CONFIGURATION = 0xFE01,
    // Add other command words as needed
};

class HLKLD2410SComponent;  // Forward declaration

class EnableConfigButton : public Component, public button::Button {
 public:
    explicit EnableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
    void press_action() override;
 protected:
    HLKLD2410SComponent *parent_;
};

class DisableConfigButton : public Component, public button::Button {
 public:
    explicit DisableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
    void press_action() override;
 protected:
    HLKLD2410SComponent *parent_;
};

class HLKLD2410SComponent : public Component, public uart::UARTDevice {
 public:
    explicit HLKLD2410SComponent(uart::UARTComponent *parent) : UARTDevice(parent) {}

    void setup() override;
    void loop() override;

    void set_presence_sensor(binary_sensor::BinarySensor *presence_sensor) { presence_sensor_ = presence_sensor; }
    void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
    void set_config_mode_sensor(binary_sensor::BinarySensor *config_mode_sensor) { config_mode_sensor_ = config_mode_sensor; }
    void add_gate_energy_sensor(uint8_t gate, sensor::Sensor *energy_sensor) { gate_energy_sensors_[gate] = energy_sensor; }
    void set_throttle(uint16_t throttle) { throttle_ = throttle; }
    void set_unmanned_delay(uint16_t delay) { unmanned_delay_ = delay; }
    void switch_output_mode(bool standard_mode) { output_mode_standard_ = standard_mode; }

    bool enable_configuration();
    bool disable_configuration();

 protected:
    void process_simple_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data);
    void process_standard_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data);
    bool write_command_(CommandWord command, const std::vector<uint8_t> &data = {});
    bool read_ack_(CommandWord expected_cmd);
    bool verify_frame_header_(const uint8_t *buf, size_t len);
    bool verify_frame_end_(const uint8_t *buf, size_t len);

    binary_sensor::BinarySensor *presence_sensor_{nullptr};
    sensor::Sensor *distance_sensor_{nullptr};
    binary_sensor::BinarySensor *config_mode_sensor_{nullptr};
    std::map<uint8_t, sensor::Sensor *> gate_energy_sensors_{};

    uint16_t throttle_{0};
    uint32_t last_update_{0};
    uint16_t unmanned_delay_{0};
    uint32_t last_presence_detected_{0};
    bool output_mode_standard_{true};
    bool config_mode_{false};
    std::vector<uint8_t> trigger_thresholds_{};
    std::vector<uint8_t> hold_thresholds_{};
};

} // namespace hlk_ld2410s
} // namespace esphome