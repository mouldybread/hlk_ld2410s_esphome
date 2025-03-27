/**
 * HLK-LD2410S mmWave Radar Sensor Component for ESPHome.
 * 
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 14:46:10 UTC
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/button/button.h"
#include <vector>
#include <map>

namespace esphome {
namespace hlk_ld2410s {

static const char *const TAG = "hlk_ld2410s";
static const uint8_t MAX_GATES = 16;

// Configuration frame constants
static const uint8_t CONFIG_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CONFIG_FRAME_END[] = {0x04, 0x03, 0x02, 0x01};
static const uint16_t CONFIG_FRAME_MIN_LENGTH = 8;
static const uint32_t ACK_TIMEOUT_MS = 1000;
static const uint32_t COMMAND_DELAY_MS = 100;

// Command words for configuration
enum class CommandWord : uint16_t {
    ENABLE_CONFIGURATION = 0xFF00,
    DISABLE_CONFIGURATION = 0xFE00,
    SWITCH_OUTPUT_MODE = 0x7A00,
    READ_FIRMWARE_VERSION = 0x0000,
    WRITE_SERIAL_NUMBER = 0x1000,
    READ_SERIAL_NUMBER = 0x1100,
    WRITE_PARAMETERS = 0x7000,
    READ_PARAMETERS = 0x7100,
    AUTO_THRESHOLD = 0x0900,
    WRITE_TRIGGER_THRESHOLD = 0x7200,
    READ_TRIGGER_THRESHOLD = 0x7300,
    WRITE_HOLD_THRESHOLD = 0x7600,
    READ_HOLD_THRESHOLD = 0x7700
};

class HLKLD2410SComponent;  // Forward declaration

class EnableConfigButton : public button::Button, public Component {
 public:
    explicit EnableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
    void press_action() override;
    void setup() override {}
    void dump_config() override {}
 protected:
    HLKLD2410SComponent *parent_;
};

class DisableConfigButton : public button::Button, public Component {
 public:
    explicit DisableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
    void press_action() override;
    void setup() override {}
    void dump_config() override {}
 protected:
    HLKLD2410SComponent *parent_;
};

class HLKLD2410SComponent : public Component, public uart::UARTDevice {
 public:
    explicit HLKLD2410SComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

    void setup() override;
    void loop() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::DATA; }

    void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
    void set_presence_sensor(binary_sensor::BinarySensor *presence_sensor) { presence_sensor_ = presence_sensor; }
    void set_config_mode_sensor(binary_sensor::BinarySensor *config_mode_sensor) { config_mode_sensor_ = config_mode_sensor; }
    void set_gate_energy_sensor(uint8_t gate, sensor::Sensor *energy_sensor) { gate_energy_sensors_[gate] = energy_sensor; }

    // Configuration setters
    void set_throttle(uint32_t throttle) { throttle_ = throttle; }
    void set_output_mode(bool standard_mode) { output_mode_standard_ = standard_mode; }
    void set_response_speed(uint8_t speed) { response_speed_ = (speed == 10) ? 10 : 5; }
    void set_unmanned_delay(uint16_t delay) { unmanned_delay_ = std::min(std::max(delay, uint16_t(10)), uint16_t(120)); }
    void set_status_report_frequency(float freq) { status_report_freq_ = std::min(std::max(freq, 0.5f), 8.0f); }
    void set_distance_report_frequency(float freq) { distance_report_freq_ = std::min(std::max(freq, 0.5f), 8.0f); }
    void set_farthest_gate(uint8_t gate) { farthest_gate_ = std::min(gate, uint8_t(16)); }
    void set_nearest_gate(uint8_t gate) { nearest_gate_ = std::min(gate, uint8_t(16)); }
    void set_trigger_thresholds(const std::vector<uint8_t> &thresholds) { trigger_thresholds_ = thresholds; }
    void set_hold_thresholds(const std::vector<uint8_t> &thresholds) { hold_thresholds_ = thresholds; }
    void set_auto_threshold(uint8_t trigger_factor, uint8_t hold_factor, uint8_t scan_time) {
        auto_threshold_trigger_ = trigger_factor;
        auto_threshold_hold_ = hold_factor;
        auto_threshold_scan_ = scan_time;
    }

    // Button setters
    void set_enable_config_button(EnableConfigButton *button) { enable_config_button_ = button; }
    void set_disable_config_button(DisableConfigButton *button) { disable_config_button_ = button; }

    // Configuration mode control
    bool enable_configuration();
    bool disable_configuration();

 protected:
    // Command methods
    bool write_command_(CommandWord command, const std::vector<uint8_t> &data = {});
    bool read_ack_(CommandWord expected_cmd);
    bool write_parameters_();
    bool read_parameters_();
    bool write_trigger_thresholds_();
    bool read_trigger_thresholds_();
    bool write_hold_thresholds_();
    bool read_hold_thresholds_();
    bool set_auto_threshold_();
    bool switch_output_mode_(bool standard_mode);

    // Utility methods
    void process_simple_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data);
    void process_standard_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data);
    bool verify_frame_header_(const uint8_t *buf, size_t len);
    bool verify_frame_end_(const uint8_t *buf, size_t len);
    void dump_data_(const char* prefix, const uint8_t* data, size_t len);
    void check_uart_settings_();

    // Sensors
    binary_sensor::BinarySensor *presence_sensor_{nullptr};
    sensor::Sensor *distance_sensor_{nullptr};
    binary_sensor::BinarySensor *config_mode_sensor_{nullptr};
    std::map<uint8_t, sensor::Sensor *> gate_energy_sensors_{};

    // Configuration buttons
    EnableConfigButton *enable_config_button_{nullptr};
    DisableConfigButton *disable_config_button_{nullptr};

    // Configuration parameters
    uint32_t throttle_{50};
    uint32_t last_update_{0};
    uint32_t last_presence_detected_{0};
    bool output_mode_standard_{true};
    bool config_mode_{false};
    
    // Advanced configuration
    uint8_t response_speed_{5};  // 5 = Normal, 10 = Fast
    uint16_t unmanned_delay_{40};  // 10-120 seconds
    float status_report_freq_{0.5f};  // 0.5-8Hz in 0.5Hz steps
    float distance_report_freq_{0.5f};  // 0.5-8Hz in 0.5Hz steps
    uint8_t farthest_gate_{12};  // 1-16
    uint8_t nearest_gate_{0};  // 0-16
    uint8_t auto_threshold_trigger_{2};
    uint8_t auto_threshold_hold_{1};
    uint8_t auto_threshold_scan_{120};
    std::vector<uint8_t> trigger_thresholds_{};
    std::vector<uint8_t> hold_thresholds_{};
};

}  // namespace hlk_ld2410s
}  // namespace esphome