/**
 * Copyright (c) 2021 Brocksdorff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <string>
#include <deque>
#include <vector>
#ifdef ARDUINO_ARCH_ESP32
#include <AsyncTCP.h>
#endif
#ifdef ARDUINO_ARCH_ESP8266
#include <ESPAsyncTCP.h>
#endif

namespace esphome {
namespace gcodestreamer {

class GCodeStreamer : public Component, public uart::UARTDevice, public binary_sensor::BinarySensor {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;

  // void update() {};
  void set_port(uint16_t port) { this->port_ = port; }

  const uint8_t MAX_QUEUE = 16;
  const uint8_t MAX_COMMAND_LENGTH = 95;
  const uint16_t DEFAULT_PORT = 1212;
  const uint32_t ONE_MINUTE = 60 * 1000UL;
  const uint32_t TCP_IDLE_TIME = 5 * ONE_MINUTE;
  const uint32_t SERIAL_TIMEOUT = 3 * 1000UL;  // 3 sec
  const std::string DEBUG = "debug";

 protected:
  void maintenance_();
  void debug_();
  void write_to_serial_();
  void read_from_serial_();
  static uint8_t checksum_line(const std::string& line);
  std::string build_line_(uint32_t line_number, const std::string& line);
  void reset_();
  void reset_printer_();
  void read_buffer_();
  void serial_in_(const std::string& line);
  void relay_serial_in_(const std::string& line);
  void relay_serial_out_(const std::string& line);

  // speed up
  esphome::HighFrequencyLoopRequester high_freq_;

  // private method to read stream with timeout
  int timed_read_();
  std::string read_line_();

  AsyncServer server_{0};
  AsyncClient* client_{nullptr};
  bool client_connected_{false};
  uint16_t port_{DEFAULT_PORT};
  std::vector<uint8_t> recv_buf_{};
  std::deque<std::string> send_queue_{};
  std::string line_to_send_{"\n"};
  bool ready_to_send_flag_{true};
  bool resend_last_line_flag_{false};
  bool relay_serial_in_flag_{false};
  bool relay_serial_out_flag_{false};
  bool quiet_flag_{false};
  bool no_exit_flag_{false};
  bool connecting_flag_{false};
  bool disconnecting_flag_{false};
  uint16_t optimistic_resends_{0};
  uint32_t printer_timeout_{0};
  uint32_t line_number_{0};
  uint32_t last_send_serial_{0};
  uint32_t last_recv_serial_{0};
  uint32_t last_recv_tcp_{0};
  uint16_t loops_{0};
};

}  // namespace gcodestreamer
}  // namespace esphome
