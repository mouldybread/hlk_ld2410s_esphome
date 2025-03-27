#include "hlk_ld2410s.h"

void HLK_LD2410S::parse_byte(uint8_t byte) {
  static uint8_t buffer[64];
  static uint8_t index = 0;

  buffer[index++] = byte;

  if (index == 64) {
    index = 0;
    return;
  }

  if (index < 4) {
    return;
  }

  uint8_t frame_head = buffer[0];
  if (frame_head != 0x6E && frame_head != 0xF4) {
    index = 0;
    return;
  }

  uint8_t frame_end = buffer[index - 1];
  if (frame_head == 0x6E && frame_end != 0x62) {
    return;
  }

  if (frame_head == 0xF4 && frame_end != 0xF8) {
    return;
  }

  if (frame_head == 0x6E) {
    // Minimal data format
    uint8_t target_state = buffer[1];
    uint16_t object_distance = buffer[2] | (buffer[3] << 8);

    this->distance_sensor->publish_state(object_distance);
    this->presence_sensor->publish_state(target_state == 2 || target_state == 3);
  } else if (frame_head == 0xF4) {
    // Standard data format
    uint8_t target_state = buffer[4];
    uint16_t object_distance = buffer[5] | (buffer[6] << 8);

    this->distance_sensor->publish_state(object_distance);
    this->presence_sensor->publish_state(target_state == 2 || target_state == 3);
  }

  index = 0;
}