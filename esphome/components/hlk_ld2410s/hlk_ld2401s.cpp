#include "hlk_ld2410s.h"

namespace esphome {
namespace hlk_ld2410s {

const uint8_t FRAME_HEAD_MINIMAL = 0x6E;
const uint8_t FRAME_HEAD_STANDARD = 0xF4;
const uint8_t FRAME_END_MINIMAL = 0x62;
const uint8_t FRAME_END_STANDARD = 0xF8;
const uint8_t BUFFER_SIZE = 64;

void HLK_LD2410S::setup() {
  // Setup code here
}

void HLK_LD2410S::loop() {
  while (available() > 0) {
    uint8_t byte;
    if (!read_byte(&byte)) {
      // Handle read error
      continue;
    }
    parse_byte(byte);
  }
}

void HLK_LD2410S::parse_byte(uint8_t byte) {
  static uint8_t buffer[BUFFER_SIZE];
  static uint8_t index = 0;

  buffer[index++] = byte;

  if (index >= BUFFER_SIZE) {
    index = 0;
    return;
  }

  if (index < 4) {
    return;
  }

  uint8_t frame_head = buffer[0];
  if (frame_head != FRAME_HEAD_MINIMAL && frame_head != FRAME_HEAD_STANDARD) {
    index = 0;
    return;
  }

  uint8_t frame_end = buffer[index - 1];
  if ((frame_head == FRAME_HEAD_MINIMAL && frame_end != FRAME_END_MINIMAL) ||
      (frame_head == FRAME_HEAD_STANDARD && frame_end != FRAME_END_STANDARD)) {
    return;
  }

  if (frame_head == FRAME_HEAD_MINIMAL) {
    // Minimal data format
    uint8_t target_state = buffer[1];
    uint16_t object_distance = buffer[2] | (buffer[3] << 8);

    this->distance_sensor->publish_state(object_distance);
    this->presence_sensor->publish_state(target_state == 2 || target_state == 3);
  } else if (frame_head == FRAME_HEAD_STANDARD) {
    // Standard data format
    uint8_t target_state = buffer[4];
    uint16_t object_distance = buffer[5] | (buffer[6] << 8);

    this->distance_sensor->publish_state(object_distance);
    this->presence_sensor->publish_state(target_state == 2 || target_state == 3);
  }

  index = 0;
}

}  // namespace hlk_ld2410s
}  // namespace esphome
