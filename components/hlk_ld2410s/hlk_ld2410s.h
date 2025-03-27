#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace hlk_ld2410s {

static const uint8_t MAX_GATES = 16;
static const uint32_t ACK_TIMEOUT_MS = 1000;
static const uint8_t CONFIG_FRAME_MIN_LENGTH = 10;

class HLKLD2410SComponent;

class EnableConfigButton : public button::Button, public Component {
 public:
    explicit EnableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
    void press() override;
 protected:
    HLKLD2410SComponent *parent_;
};

class DisableConfigButton : public button::Button, public Component {
 public:
    explicit DisableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
    void press() override;
 protected:
    HLKLD2410SComponent *parent_;
};

enum class CommandWord : uint16_t {
    ENABLE_CONFIGURATION = 0xFF01,
    DISABLE_CONFIGURATION = 0xFF02,
    READ_FIRMWARE_VERSION = 0xFFE1,
    SET_DISTANCE_GATES = 0xFFE2,
    SET_TRIGGER_THRESHOLD = 0xFFE3,
    READ_TRIGGER_THRESHOLD = 0xFFE4,
    SET_HOLD_THRESHOLD = 0xFFE5,
    READ_HOLD_THRESHOLD = 0xFFE6,
    SET_AUTO_THRESHOLD = 0xFFE7,
    SET_RESPONSE_SPEED = 0xFFE8,
};

class HLKLD2410SComponent : public Component, public uart::UARTDevice {
 public:
    explicit HLKLD2410SComponent(uart::UARTComponent *parent) : UARTDevice(parent) {}

    void setup() override;
    void loop() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::DATA; }

    void set_distance_sensor(sensor::Sensor *distance) { distance_sensor_ = distance; }
    void set_presence_sensor(binary_sensor::BinarySensor *presence) { presence_sensor_ = presence; }
    void set_config_mode_sensor(binary_sensor::BinarySensor *config_mode) { config_mode_sensor_ = config_mode; }
    void set_enable_config_button(EnableConfigButton *enable_config) { enable_config_button_ = enable_config; }
    void set_disable_config_button(DisableConfigButton *disable_config) { disable_config_button_ = disable_config; }

    void switch_output_mode(bool standard_mode) { output_mode_standard_ = standard_mode; }
    void set_response_speed(uint8_t speed) { response_speed_ = speed; }
    void set_throttle(uint32_t throttle) { throttle_ = throttle; }
    void set_unmanned_delay(uint8_t delay) { unmanned_delay_ = delay; }
    void set_status_report_frequency(float freq) { status_report_frequency_ = freq; }
    void set_distance_report_frequency(float freq) { distance_report_frequency_ = freq; }
    void set_farthest_gate(uint8_t gate) { farthest_gate_ = gate; }
    void set_nearest_gate(uint8_t gate) { nearest_gate_ = gate; }
    void set_trigger_thresholds(const std::vector<uint8_t> &thresholds);
    void set_hold_thresholds(const std::vector<uint8_t> &thresholds);
    void set_auto_threshold(uint8_t trigger_factor, uint8_t hold_factor, uint8_t scan_time);
    void set_gate_energy_sensor(uint8_t gate, sensor::Sensor *energy);

    bool enable_configuration();
    bool disable_configuration();
    bool read_firmware_version();
    bool write_trigger_threshold(const std::vector<uint8_t> &thresholds);
    bool read_trigger_threshold();
    bool write_hold_threshold(const std::vector<uint8_t> &thresholds);
    bool read_hold_threshold();

 protected:
    sensor::Sensor *distance_sensor_{nullptr};
    binary_sensor::BinarySensor *presence_sensor_{nullptr};
    binary_sensor::BinarySensor *config_mode_sensor_{nullptr};
    EnableConfigButton *enable_config_button_{nullptr};
    DisableConfigButton *disable_config_button_{nullptr};
    std::map<uint8_t, sensor::Sensor *> gate_energy_sensors_{};

    bool output_mode_standard_{true};
    uint8_t response_speed_{5};
    uint32_t throttle_{50};
    uint8_t unmanned_delay_{0};
    float status_report_frequency_{0.5f};
    float distance_report_frequency_{0.5f};
    uint8_t farthest_gate_{12};
    uint8_t nearest_gate_{0};
    std::vector<uint8_t> trigger_thresholds_;
    std::vector<uint8_t> hold_thresholds_;

    uint32_t last_presence_detected_{0};
    bool config_mode_{false};
    void process_standard_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data);
    void process_simple_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data);
    bool read_ack_(CommandWord expected_command);
    bool write_command_(CommandWord command, const std::vector<uint8_t> &data = {});
};

}  // namespace hlk_ld2410s
}  // namespace esphome