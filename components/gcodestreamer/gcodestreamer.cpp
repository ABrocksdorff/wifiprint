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

#include "gcodestreamer.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

namespace esphome {
namespace gcodestreamer {

static const char *TAG = "GCodeStreamer";

void GCodeStreamer::setup() {
  ESP_LOGCONFIG(TAG, "Setting up GCodeStreamer ...");
  this->server_ = AsyncServer(this->port_);
  this->server_.begin();
  this->server_.onClient(
      [this](void *h, AsyncClient *new_client) {
        if (new_client == nullptr)
          return;

        if (this->client_ == nullptr) {
          this->client_ = new_client;
          this->connecting_flag_ = true;
          this->reset_();

          new_client->onDisconnect(
              [this](void *h, AsyncClient *client) {
                this->client_connected_ = false;
                this->disconnecting_flag_ = true;
              },
              nullptr);

          new_client->onData(
              [this](void *h, AsyncClient *client, void *data, size_t len) {
                if (len == 0 || data == nullptr)
                  return;

                client->ackLater();
                auto *buf = static_cast<uint8_t *>(data);
                this->recv_buf_.insert(this->recv_buf_.end(), buf, buf + len);
                this->last_recv_tcp_ = millis();
              },
              nullptr);

        } else {
          new_client->onDisconnect(
              [](void *h, AsyncClient *client) {
                delete client;  // dont care
              },
              nullptr);

          new_client->write("Another client already connected - Bye Bye\n");
          new_client->close(true);
        }
      },
      this);

  this->set_interval(10000, [this]() { this->maintenance_(); });
  this->publish_initial_state(false);

  // reset on boot
  this->reset_printer_();
}

void GCodeStreamer::loop() {
  this->loops_++;
  if (this->connecting_flag_) {
    this->connecting_flag_ = false;
    if (this->client_ == nullptr) {
      ESP_LOGE(TAG, "Client connecting is NULL");
    } else {
      this->client_connected_ = true;
      this->publish_state(true);
      this->client_->write("GCodeStreamer v1.2 ready\n");
      this->high_freq_.start();  // full throttle
      this->last_recv_tcp_ = millis();
      ESP_LOGI(TAG, "New client connected from %s", this->client_->remoteIP().toString().c_str());
    }
  } else if (this->disconnecting_flag_) {
    this->disconnecting_flag_ = false;
    delete this->client_;
    this->client_ = nullptr;  // explicit set
    this->high_freq_.stop();
    this->publish_state(false);
    this->cancel_interval(DEBUG);
    this->reset_printer_();
    ESP_LOGI(TAG, "Client disconnected");
  }
  this->read_from_serial_();
  this->write_to_serial_();
  this->read_buffer_();
}

// from Stream.h private method to read stream with timeout
auto GCodeStreamer::timed_read_() -> int {
  int c = 0;
  uint32_t start_millis = millis();
  do {
    c = this->read();
    if (c >= 0) {
      return c;
    }
    if (_timeout == 0) {
      return -1;
    }
    delay(0);
  } while (millis() - start_millis < 100UL);
  return -1;  // -1 indicates timeout
}

// from Stream.h
auto GCodeStreamer::read_line_() -> std::string {
  std::string ret;
  int c = this->timed_read_();
  while (c >= 0 && c != '\n' && ret.length() < MAX_COMMAND_LENGTH) {
    ret += (char) c;
    c = this->timed_read_();
  }
  return ret;
}

void GCodeStreamer::read_from_serial_() {
  while (this->available() > 2) {
    const std::string line = this->read_line_();
    if (this->relay_serial_in_flag_) {
      this->relay_serial_in_(line);
    } else if (!this->quiet_flag_) {
      this->serial_in_(line);
    }
    this->last_recv_serial_ = millis();
    if (str_startswith(line, "ok")) {
      // ok
      this->ready_to_send_flag_ = true;
      this->optimistic_resends_ = 0;
    } else if (str_startswith(line, "echo:busy:")) {
      // echo:busy: processing
      this->printer_timeout_ = this->last_recv_serial_ + SERIAL_TIMEOUT;
    } else if (str_startswith(line, "Resend:")) {
      // Resend: N
      this->resend_last_line_flag_ = true;
      std::string to_parse = line.substr(8);
      int value = strtol(to_parse.c_str(), nullptr, 10);
      if (value >= 0L) {
        this->line_number_ = value;
        ESP_LOGI(TAG, "Resend with line number: %i", this->line_number_);
      } else {
        ESP_LOGW(TAG, "Resend failed to parse line number: %s", to_parse.c_str());
      }
    } else if (str_startswith(line, "wait")) {
      // wait
      // either our line got lost or we didnt got ok
      if (!this->ready_to_send_flag_) {
        this->ready_to_send_flag_ = true;
        this->printer_timeout_ = this->last_recv_serial_ + SERIAL_TIMEOUT;
        this->resend_last_line_flag_ = true;
        if (this->line_number_ > 0) {
          // assume our line got lost
          this->line_number_--;
        }
        ESP_LOGW(TAG, "Optimistic resend after wait: %s", this->line_to_send_.c_str());
        this->optimistic_resends_++;
      }
    } else if (str_startswith(line, "Error")) {
      ESP_LOGW(TAG, "%s", line.c_str());  // ALWAYS LOG
      if (str_startswith(line, "Error:checksum")) {
        // Error:checksum mismatch, Last Line: 1018
        this->line_number_ = strtol(line.substr(36).c_str(), nullptr, 10) + 1;
        this->resend_last_line_flag_ = true;
        ESP_LOGW(TAG, "Resend after checksum mismatch: %s", this->line_to_send_.c_str());
      } else if (str_startswith(line, "Error:Line")) {
        // Error:Line Number is not Last Line Number+1, Last Line: 0
        this->line_number_ = strtol(line.substr(56).c_str(), nullptr, 10) + 1;
        this->resend_last_line_flag_ = true;
        this->ready_to_send_flag_ = true;
        this->printer_timeout_ = this->last_recv_serial_ + SERIAL_TIMEOUT;
        ESP_LOGW(TAG, "Resend after line number mismatch: %s", this->line_to_send_.c_str());
      }
    }
    delay(0);
  }
}

void GCodeStreamer::serial_in_(const std::string &line) {
  if (this->client_connected_) {
    std::string tosend(line);
    tosend.append("\n");
    this->client_->write(tosend.c_str());
  }
}

void GCodeStreamer::relay_serial_in_(const std::string &line) {
  if (this->client_connected_) {
    std::string tosend(to_string(this->last_recv_serial_));
    tosend.append(" recv ");
    this->client_->write(tosend.c_str());
    this->serial_in_(line);
  }
}

void GCodeStreamer::relay_serial_out_(const std::string &line) {
  if (this->client_connected_) {
    std::string tosend(to_string(this->last_send_serial_));
    tosend.append(" send ");
    tosend.append(line);
    tosend.append("\n");
    this->client_->write(tosend.c_str());
  }
}

void GCodeStreamer::debug_() {
  // this is somewhat problematic
  if (this->client_connected_) {
    std::string tosend(to_string(to_string(millis())));
    tosend.append(" dbug bs ");
    tosend.append(to_string(this->recv_buf_.size()));
    tosend.append(" qs ");
    tosend.append(to_string(this->send_queue_.size()));
    tosend.append(" fh ");
    tosend.append(to_string(ESP.getFreeHeap()));
    tosend.append(" l# ");
    tosend.append(to_string(this->line_number_));
    tosend.append(" lc ");
    tosend.append(to_string(this->loops_));
    tosend.append(" rs ");
    tosend.append(to_string(this->last_recv_serial_));
    tosend.append(" ss ");
    tosend.append(to_string(this->last_send_serial_));
    tosend.append(" rt ");
    tosend.append(to_string(this->last_recv_tcp_));
    tosend.append("\n");
    this->client_->write(tosend.c_str());
  }
  this->loops_ = 0;
}

void GCodeStreamer::maintenance_() {
  if (this->client_connected_) {
    if (this->send_queue_.empty()) {
      if (millis() > this->last_recv_tcp_ + TCP_IDLE_TIME) {
        if (this->no_exit_flag_)
          return;
        // no input,queue empty
        this->client_->write("Close idle connection\n");
        this->client_->close();
        ESP_LOGI(TAG, "Close idle connection");
      }
    } else {
      if (this->optimistic_resends_ >= 5) {
        // printer gone
        this->client_->write("Printer off-line, close connection\n");
        this->client_->close();
        ESP_LOGI(TAG, "Printer off-line, close connection");
      }
    }
  }
}

void GCodeStreamer::read_buffer_() {
  if (this->send_queue_.size() < MAX_QUEUE) {
    if (!this->recv_buf_.empty()) {
      auto it_line = std::find(this->recv_buf_.begin(), this->recv_buf_.end(), '\n');
      if (it_line != this->recv_buf_.end()) {
        std::string line(this->recv_buf_.begin(), it_line);

        std::advance(it_line, 1);
        this->recv_buf_.erase(this->recv_buf_.begin(), it_line);

        if (this->client_ != nullptr) {
          this->client_->ack(it_line - this->recv_buf_.begin());
        }

        // remove blank and *
        line.erase(
            std::remove_if(line.begin(), line.end(), [](unsigned char x) { return std::isspace(x) || x == '*'; }),
            line.end());

        if (line.empty())
          return;

        if (str_startswith(line, ";")) {
          if (str_startswith(line, ";RELAYIN")) {
            this->relay_serial_in_flag_ = true;
          } else if (str_startswith(line, ";RELAYOUT")) {
            this->relay_serial_out_flag_ = true;
          } else if (str_startswith(line, ";DEBUG")) {
            this->loops_ = 0;
            this->set_interval(DEBUG, 5000, [this]() { this->debug_(); });
          } else if (str_startswith(line, ";QUIET")) {
            this->quiet_flag_ = true;
          } else if (str_startswith(line, ";EXIT")) {
            if (this->client_connected_) {
              this->client_->write("Bye Bye\n");
              this->client_->close();
            }
          } else if (str_startswith(line, ";NORELAYIN")) {
            this->relay_serial_in_flag_ = false;
          } else if (str_startswith(line, ";NORELAYOUT")) {
            this->relay_serial_out_flag_ = false;
          } else if (str_startswith(line, ";NODEBUG")) {
            this->cancel_interval(DEBUG);
          } else if (str_startswith(line, ";NOQUIET")) {
            this->quiet_flag_ = false;
          } else if (str_startswith(line, ";NOEXIT")) {
            this->no_exit_flag_ = true;
          }

          // comments
          return;
        }

        // ALL CAPS
        std::transform(line.begin(), line.end(), line.begin(), ::toupper);

        // LCD message is problematic
        if (str_startswith(line, "M117"))
          return;

        auto it_comment = std::find(line.begin(), line.end(), ';');
        if (it_comment == line.end()) {
          this->send_queue_.emplace_back(line);
        } else {
          this->send_queue_.emplace_back(line.begin(), it_comment);
        }
      } else {
        // so far no cmd
        if (this->recv_buf_.size() > MAX_COMMAND_LENGTH) {
          ESP_LOGW(TAG, "Line too long");
          if (this->client_connected_) {
            this->client_->write("Line too long\n");
            this->client_->close();
          }
        }
      }
    }
  }
}

void GCodeStreamer::write_to_serial_() {
  if (this->ready_to_send_flag_) {
    if (this->resend_last_line_flag_) {
      std::string line_send = build_line_(this->line_number_, this->line_to_send_);
      this->write_str(line_send.c_str());
      this->write('\n');
      this->last_send_serial_ = millis();
      this->printer_timeout_ = this->last_send_serial_ + SERIAL_TIMEOUT;
      if (this->relay_serial_out_flag_)
        this->relay_serial_out_(line_send);
      this->resend_last_line_flag_ = false;
      this->ready_to_send_flag_ = false;
      this->line_number_++;
    } else {
      if (!this->send_queue_.empty()) {
        this->line_to_send_ = this->send_queue_.front();
        this->send_queue_.pop_front();
        std::string line_send = build_line_(this->line_number_, this->line_to_send_);
        this->write_str(line_send.c_str());
        this->write('\n');
        this->last_send_serial_ = millis();
        this->printer_timeout_ = this->last_send_serial_ + SERIAL_TIMEOUT;
        if (this->relay_serial_out_flag_)
          this->relay_serial_out_(line_send);
        this->ready_to_send_flag_ = false;
        this->line_number_++;
      }
    }
  } else if (millis() > this->printer_timeout_ && this->optimistic_resends_ < 5) {
    // printer timeout
    this->ready_to_send_flag_ = true;
    this->resend_last_line_flag_ = true;
    if (this->line_number_ > 0) {
      // assume our line got lost
      this->line_number_--;
    }
    ESP_LOGW(TAG, "Optimistic resend after timeout: %s", this->line_to_send_.c_str());
    this->optimistic_resends_++;
  }
}

void GCodeStreamer::reset_() {
  this->flush();

  this->line_to_send_ = "\n";
  this->ready_to_send_flag_ = true;
  this->resend_last_line_flag_ = false;
  this->relay_serial_in_flag_ = false;
  this->relay_serial_out_flag_ = false;
  this->quiet_flag_ = false;
  this->no_exit_flag_ = false;
  this->optimistic_resends_ = 0;

  // clear buffer
  this->recv_buf_.clear();
  // clear queue
  this->send_queue_.clear();
  // reset line number
  this->line_number_ = 0;
  this->send_queue_.emplace_back("M110");
}

void GCodeStreamer::reset_printer_() {
  this->reset_();
  // turn off heater
  this->send_queue_.emplace_back("M140S0");
  // turn off heater
  this->send_queue_.emplace_back("M104S0");
  // Turn off one of the fans. If no fan index is given, the print cooling fan.
  this->send_queue_.emplace_back("M107");
  // home only X
  this->send_queue_.emplace_back("G28X");
}

auto GCodeStreamer::build_line_(uint32_t line_number, const std::string &line) -> std::string {
  std::string line_out("N");
  auto ln_s = to_string(line_number);
  line_out.append(ln_s);
  line_out.append(line);
  uint8_t cs = esphome::gcodestreamer::GCodeStreamer::checksum_line(line_out);
  auto cs_s = to_string(cs);
  line_out.append("*");
  line_out.append(cs_s);
  return line_out;
}

auto GCodeStreamer::checksum_line(const std::string &line) -> uint8_t {
  uint8_t checksum = 0;
  uint8_t count = line.length();
  while (count)
    checksum ^= line.at(--count);
  return checksum;
}

void GCodeStreamer::dump_config() {
  ESP_LOGCONFIG(TAG, "GCodeStreamer:");
  ESP_LOGCONFIG(TAG, "  Address: %s:%u", network_get_address().c_str(), this->port_);
}

void GCodeStreamer::on_shutdown() {
  if (this->client_connected_) {
    this->client_->write("GCodeStreamer shutdown\n");
    this->client_->close(true);
  }
}

}  // namespace gcodestreamer
}  // namespace esphome
