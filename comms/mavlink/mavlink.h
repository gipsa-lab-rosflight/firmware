/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSFLIGHT_FIRMWARE_MAVLINK_H
#define ROSFLIGHT_FIRMWARE_MAVLINK_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wcast-align"
#include "v1.0/rosflight/mavlink.h"
# pragma GCC diagnostic pop

#include "comm_link.h"
#include "board.h"

namespace rosflight_firmware
{

class Board;

class Mavlink : public CommLink
{
public:
  Mavlink(Board &board);
  void init(uint32_t baud_rate, uint32_t dev) override;
  void receive() override;

  void send_attitude_quaternion(uint8_t system_id,
                                uint64_t timestamp_us,
                                const turbomath::Quaternion &attitude,
                                const turbomath::Vector &angular_velocity) override;
  void send_baro(uint8_t system_id, float altitude, float pressure, float temperature) override;
  void send_command_ack(uint8_t system_id, Command command, bool success) override;
  void send_diff_pressure(uint8_t system_id, float velocity, float pressure, float temperature) override;
  void send_heartbeat(uint8_t system_id, bool fixed_wing) override;
  void send_imu(uint8_t system_id, uint64_t timestamp_us,
                const turbomath::Vector &accel,
                const turbomath::Vector &gyro,
                float temperature) override;
  void send_log_message(uint8_t system_id, LogSeverity severity, const char *text) override;
  void send_mag(uint8_t system_id, const turbomath::Vector &mag) override;
  void send_named_value_int(uint8_t system_id, uint32_t timestamp_ms, const char *const name, int32_t value) override;
  void send_named_value_float(uint8_t system_id, uint32_t timestamp_ms, const char *const name, float value) override;
  void send_output_raw(uint8_t system_id, uint32_t timestamp_ms, const float raw_outputs[8]) override;
  void send_param_value_int(uint8_t system_id,
                            uint16_t index,
                            const char *const name,
                            int32_t value,
                            uint16_t param_count) override;
  void send_param_value_float(uint8_t system_id,
                              uint16_t index,
                              const char *const name,
                              float value,
                              uint16_t param_count) override;
  void send_rc_raw(uint8_t system_id, uint32_t timestamp_ms, const uint16_t channels[8]) override;
  void send_sonar(uint8_t system_id, /* TODO enum type*/uint8_t type, float range, float max_range,
                  float min_range) override;
  void send_status(uint8_t system_id,
                   bool armed,
                   bool failsafe,
                   bool rc_override,
                   bool offboard,
                   uint8_t error_code,
                   uint8_t control_mode,
                   int16_t num_errors,
                   int16_t loop_time_us) override;
  void send_timesync(uint8_t system_id, int64_t tc1, int64_t ts1) override;
  void send_version(uint8_t system_id, const char *const version) override;
  virtual void send_gnss(uint8_t system_id, uint32_t time_of_week, uint8_t fix_type, uint64_t time, uint64_t nanos,
                         int32_t lat,
                         int32_t lon, int32_t height, int32_t vel_n, int32_t vel_e, int32_t vel_d, uint32_t h_acc, uint32_t v_acc,
                         int32_t ecef_x, int32_t ecef_y, int32_t ecef_z, uint32_t p_acc, int32_t ecef_v_x, int32_t ecef_v_y,
                         int32_t ecef_v_z, uint32_t s_acc, uint64_t rosflight_timestamp);
  void send_gnss_raw(uint8_t system_id, uint32_t time_of_week, uint16_t year, uint8_t month, uint8_t day,
                     uint8_t hour, uint8_t min, uint8_t sec, uint8_t valid, uint32_t t_acc,
                     int32_t nano, uint8_t fix_type, uint8_t num_sat,
                     int32_t lon, int32_t lat, int32_t height, int32_t height_msl,
                     uint32_t h_acc, uint32_t v_acc, int32_t vel_n, int32_t vel_e,
                     int32_t vel_d, int32_t g_speed, int32_t head_mot, uint32_t s_acc,
                     uint32_t head_acc, uint16_t p_dop, uint64_t rosflight_timestamp);
	void send_multi_range(uint8_t system_id, uint8_t nb_ranges, const uint16_t* ranges) override;
	void send_battery(uint8_t system_id, float voltage, float percent) override;
  void send_error_data(uint8_t system_id, const BackupData &error_data);

private:
  void send_message(const mavlink_message_t &msg);

  void handle_msg_param_request_list(const mavlink_message_t *const msg);
  void handle_msg_param_request_read(const mavlink_message_t *const msg);
  void handle_msg_param_set(const mavlink_message_t *const msg);
  void handle_msg_offboard_control(const mavlink_message_t *const msg);
  void handle_msg_attitude_correction(const mavlink_message_t *const msg);
  void handle_msg_rosflight_cmd(const mavlink_message_t *const msg);
  void handle_msg_timesync(const mavlink_message_t *const msg);
  void handle_msg_heartbeat(const mavlink_message_t *const msg);
  void handle_mavlink_message(void);

  Board &board_;

  uint32_t compid_ = 250;
  mavlink_message_t in_buf_;
  mavlink_status_t status_;
  bool initialized_ = false;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_MAVLINK_H
