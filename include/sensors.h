/*
 * Copyright (c) 2017, James Jackson, Daniel Koch, and Craig Bidstrup,
 * BYU MAGICC Lab
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

#ifndef ROSFLIGHT_FIRMWARE_SENSORS_H
#define ROSFLIGHT_FIRMWARE_SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include <turbomath/turbomath.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" // Ignore warning about leaving struct fields blank

namespace rosflight_firmware
{
// Fix type, as defined in sensor_msgs/NavSatStatus
typedef enum
{
  NO_FIX,   // Unable to fix position
  FIX,      // Unaugmented fix
  SBAS_FIX, // with satellite-based augmentation
  GBAS_FIX  // with ground-based augmentation
} GNSSFixType;

#pragma GCC diagnostic push // Allow anonymous nested unions and structs
#pragma GCC diagnostic ignored "-Wpedantic"

#define CELL_VOLTAGE_PERCENT_SAMPLES 13

struct GNSSData
{
  GNSSFixType fix_type;
  uint32_t time_of_week;
  uint64_t time; // Unix time, in seconds
  uint64_t nanos; // Fractional time
  int32_t lat; // deg*10^-7
  int32_t lon; // deg*10^-7
  int32_t height; // mm
  int32_t vel_n; // mm/s
  int32_t vel_e; // mm/s
  int32_t vel_d; // mm/s
  uint32_t h_acc; // mm
  uint32_t v_acc; // mm
  struct
  {
    int32_t x; // cm
    int32_t y; // cm
    int32_t z; // cm
    uint32_t p_acc; // cm
    int32_t vx; // cm/s
    int32_t vy; // cm/s
    int32_t vz; // cm/s
    uint32_t s_acc; // cm/s
  } ecef;
  uint64_t rosflight_timestamp; // microseconds, time stamp of last byte in the message
};

struct GNSSRaw
{
  uint64_t time_of_week;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t t_acc;
  int32_t nano;
  uint8_t fix_type;
  uint8_t num_sat;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t height_msl;
  uint32_t h_acc;
  uint32_t v_acc;
  int32_t vel_n;
  int32_t vel_e;
  int32_t vel_d;
  int32_t g_speed;
  int32_t head_mot;
  uint32_t s_acc;
  uint32_t head_acc;
  uint16_t p_dop;
  uint64_t rosflight_timestamp; // microseconds, time stamp of last byte in the message
};

#define MULTI_RANGE_MAX_RANGES 16
struct MultiRangeData
{
	uint8_t nbRanges;
	uint16_t ranges[MULTI_RANGE_MAX_RANGES];
};

#pragma GCC diagnostic pop

class ROSflight;

class Sensors
{
public:
  struct Data
  {
    turbomath::Vector accel = {0, 0, 0};
    turbomath::Vector gyro = {0, 0, 0};
    turbomath::Quaternion fcu_orientation = {1, 0, 0, 0};
    float imu_temperature = 0;
    uint64_t imu_time = 0;

    float diff_pressure_velocity = 0;
    float diff_pressure = 0;
    float diff_pressure_temp = 0;
    bool diff_pressure_valid = false;

    float baro_altitude = 0;
    float baro_pressure = 0;
    float baro_temperature = 0;
    bool baro_valid = false;

    float sonar_range = 0;
    bool sonar_range_valid = false;

    GNSSData gnss_data = {};
    bool gnss_new_data = false;
    float gps_CNO = 0; // What is this?
    bool gnss_present = false;
    GNSSRaw gnss_raw = {};

    turbomath::Vector mag = {0, 0, 0};

    bool baro_present = false;
    bool mag_present = false;
    bool sonar_present = false;
    bool diff_pressure_present = false;

		bool multi_range_present = false;
		MultiRangeData multi_range_data;
		
		bool voltage_present = false;
		float voltage = 0.f;
		float battery_percent = 0.f;
  };

  Sensors(ROSflight &rosflight);

  inline const Data &data() const
  {
    return data_;
  }
  void get_filtered_IMU(turbomath::Vector &accel, turbomath::Vector &gyro, uint64_t &stamp_us);

  // function declarations
  void init();
  bool run();
  void param_change_callback(uint16_t param_id);

  // Calibration Functions
  bool start_imu_calibration(void);
  bool start_gyro_calibration(void);
  bool start_baro_calibration(void);
  bool start_diff_pressure_calibration(void);
  bool gyro_calibration_complete(void);

  inline bool should_send_imu_data(void)
  {
    if (imu_data_sent_)
      return false;
    else
      imu_data_sent_ = true;
    return true;
  }

private:
  static const float BARO_MAX_CHANGE_RATE;
  static const float BARO_SAMPLE_RATE;
  static const float DIFF_MAX_CHANGE_RATE;
  static const float DIFF_SAMPLE_RATE;
  static const float SONAR_MAX_CHANGE_RATE;
  static const float SONAR_SAMPLE_RATE;
  static const int SENSOR_CAL_DELAY_CYCLES;
  static const int SENSOR_CAL_CYCLES;
  static const float BARO_MAX_CALIBRATION_VARIANCE;
  static const float DIFF_PRESSURE_MAX_CALIBRATION_VARIANCE;
	
	static const float CELL_MAX_VOLTAGE;
	static const float CELL_MIN_VOLTAGE;
	static const float CELL_PERCENT_VOLTAGE[CELL_VOLTAGE_PERCENT_SAMPLES];
	
  class OutlierFilter
  {
  private:
    float max_change_;
    float center_;
    int window_size_;
    bool init_ = false;

  public:
    OutlierFilter() {}
    void init(float max_change_rate, float update_rate, float center);
    bool update(float new_val, float *val);
  };

  enum : uint8_t
  {
    BAROMETER,
    GNSS,
    DIFF_PRESSURE,
    SONAR,
    MAGNETOMETER,
		MULTI_RANGE,
		BATTERY_VOLTAGE,
    NUM_LOW_PRIORITY_SENSORS
  };

  ROSflight &rf_;

  Data data_;

  float accel_[3] = {0, 0, 0};
  float gyro_[3] = {0, 0, 0};

  bool calibrating_acc_flag_ = false;
  bool calibrating_gyro_flag_ = false;
  uint8_t next_sensor_to_update_ = BAROMETER;
  void init_imu();
  void calibrate_accel(void);
  void calibrate_gyro(void);
  void calibrate_baro(void);
  void calibrate_diff_pressure(void);
  void correct_imu(void);
  void correct_mag(void);
  void correct_baro(void);
  void correct_diff_pressure(void);
  bool update_imu(void);
  void update_other_sensors(void);
  void look_for_disabled_sensors(void);
  uint32_t last_time_look_for_disarmed_sensors_ = 0;
  uint32_t last_imu_update_ms_ = 0;

	
  bool new_imu_data_;
  bool imu_data_sent_;

  // IMU calibration
  uint16_t gyro_calibration_count_ = 0;
  turbomath::Vector gyro_sum_ = {0, 0, 0};
  uint16_t accel_calibration_count_ = 0;
  turbomath::Vector acc_sum_ = {0, 0, 0};
  const turbomath::Vector gravity_ = {0.0f, 0.0f, 9.80665f};
  float acc_temp_sum_ = 0.0f;
  turbomath::Vector max_ = {-1000.0f, -1000.0f, -1000.0f};
  turbomath::Vector min_ = {1000.0f, 1000.0f, 1000.0f};

  // Filtered IMU
  turbomath::Vector accel_int_;
  turbomath::Vector gyro_int_;
  uint64_t int_start_us_;
  uint64_t prev_imu_read_time_us_;

  // Baro Calibration
  bool baro_calibrated_ = false;
  float ground_pressure_ = 0.0f;
  uint16_t baro_calibration_count_ = 0;
  uint32_t last_baro_cal_iter_ms_ = 0;
  float baro_calibration_mean_ = 0.0f;
  float baro_calibration_var_ = 0.0f;

  // Diff Pressure Calibration
  bool diff_pressure_calibrated_ = false;
  uint16_t diff_pressure_calibration_count_ = 0;
  uint32_t last_diff_pressure_cal_iter_ms_ = 0;
  float diff_pressure_calibration_mean_ = 0.0f;
  float diff_pressure_calibration_var_ = 0.0f;
	
  // Sensor Measurement Outlier Filters
  OutlierFilter baro_outlier_filt_;
  OutlierFilter diff_outlier_filt_;
  OutlierFilter sonar_outlier_filt_;
	
};

} // namespace rosflight_firmware

#pragma GCC diagnostic pop // End ignore missing field initializers in structs

#endif // ROSFLIGHT_FIRMWARE_SENSORS_H
