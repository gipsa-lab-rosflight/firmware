/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *                     and Amaury Negre, CNRS
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

#include "airbot_board.h"

#ifdef SERIAL_DEBUG
#define DBG(...)				 \
	printf(__VA_ARGS__);	 \
	delay(10)
#include "printf.h"
UART *uartPtr = NULL;
static void _putc(void *p, char c)
{
	(void)p; // avoid compiler warning about unused variable
	uartPtr->put_byte(c);
}
#else
  #define DBG(...)
#endif


namespace rosflight_firmware
{

AirbotBoard::AirbotBoard()
{
}

void AirbotBoard::init_board()
{
  systemInit();
  buzzer_.init(LED1_GPIO, LED1_PIN);
  led2_.init(LED2_GPIO, LED2_PIN);
	led2_.on();

	delay(500);
	
  //int_i2c_.init(&i2c_config[BARO_I2C]);
  ext_i2c_.init(&i2c_config[EXTERNAL_I2C]);
	
  spi1_.init(&spi_config[MPU6000_SPI]);
  spi3_.init(&spi_config[FLASH_SPI]);
  uart1_.init(&uart_config[UART1], 115200, UART::MODE_8N1);
  uart6_.init(&uart_config[UART6], 57600, UART::MODE_8N1);

#ifdef SERIAL_DEBUG
	uartPtr = &uart1_;
	init_printf(NULL, _putc);
#endif
	
	DBG("init board\n");
	
  current_serial_ = &vcp_;    //uncomment this to switch to VCP as the main output	
	//secondary_serial_device_ = SERIAL_DEVICE_VCP;

	voltage_input_.init(BAT_VOLTAGE_ADC, BAT_VOLTAGE_CHANNEL, BAT_VOLTAGE_GPIO, BAT_VOLTAGE_PIN);

	DBG("end init board\n");
}

void AirbotBoard::board_reset(bool bootloader)
{
  (void)bootloader;
  NVIC_SystemReset();
}

// clock
uint32_t AirbotBoard::clock_millis()
{
  return millis();
}

uint64_t AirbotBoard::clock_micros()
{
  return micros();
}

void AirbotBoard::clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial
void AirbotBoard::serial_init(uint32_t baud_rate, uint32_t dev)
{
	DBG("serial_init (dev %d@%d)\n", dev, baud_rate);
	
  vcp_.init();
  switch (dev)
  {
  case SERIAL_DEVICE_UART1:
    uart1_.init(&uart_config[UART1], baud_rate);
    current_serial_ = &uart1_;
    secondary_serial_device_ = SERIAL_DEVICE_UART1;
    break;
  case SERIAL_DEVICE_UART6:
    uart6_.init(&uart_config[UART6], baud_rate);
    current_serial_ = &uart6_;
    secondary_serial_device_ = SERIAL_DEVICE_UART6;
    break;
	default:
    current_serial_ = &vcp_;
    secondary_serial_device_ = SERIAL_DEVICE_VCP;
  }
}

void AirbotBoard::serial_write(const uint8_t *src, size_t len)
{
  current_serial_->write(src, len);
}

uint16_t AirbotBoard::serial_bytes_available()
{
  if (vcp_.connected() || secondary_serial_device_ == SERIAL_DEVICE_VCP)
  {
    current_serial_ = &vcp_;
  }
  else
  {
    switch (secondary_serial_device_)
    {
    case SERIAL_DEVICE_UART1:
      current_serial_ = &uart1_;
      break;
    case SERIAL_DEVICE_UART6:
      current_serial_ = &uart6_;
      break;
    default:
      // no secondary serial device
      break;
    }
  }

  return current_serial_->rx_bytes_waiting();
}

uint8_t AirbotBoard::serial_read()
{

  return current_serial_->read_byte();
}

void AirbotBoard::serial_flush()
{
  current_serial_->flush();
}


// sensors
void AirbotBoard::sensors_init()
{
  while(millis() < 500) {} // wait for sensors to boot up

	DBG("init sensors\n");
	
  imu_.init(&spi1_, MPU6000_CS_GPIO, MPU6000_CS_PIN);	
	
	baro_.init(&ext_i2c_);
  mag_.init(&ext_i2c_);
  sonar_.init(&ext_i2c_);
  airspeed_.init(&ext_i2c_);

  gnss_.init(&uart6_);
	
	multi_range_.init(&ext_i2c_);
	
}

uint16_t AirbotBoard::num_sensor_errors()
{
  return ext_i2c_.num_errors();
}

bool AirbotBoard::new_imu_data()
{
  return imu_.new_data();
}

bool AirbotBoard::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
  float read_accel[3], read_gyro[3];
  imu_.read(read_accel, read_gyro, temperature, time_us);

  accel[0] = -read_accel[1];
  accel[1] = -read_accel[0];
  accel[2] = -read_accel[2];

  gyro[0] = -read_gyro[1];
  gyro[1] = -read_gyro[0];
  gyro[2] = -read_gyro[2];

  return true;
}

void AirbotBoard::imu_not_responding_error()
{
  sensors_init();
}

bool AirbotBoard::mag_present()
{
  //mag_.update();
  return mag_.present();
}

void AirbotBoard::mag_update()
{
  mag_.update();
}

void AirbotBoard::mag_read(float mag[3])
{
  mag_.update();
  mag_.read(mag);
}
bool AirbotBoard::baro_present()
{
  //baro_.update();
  return baro_.present();
}

void AirbotBoard::baro_update()
{
  baro_.update();
}

void AirbotBoard::baro_read(float *pressure, float *temperature)
{
  baro_.update();
  baro_.read(pressure, temperature);
}

bool AirbotBoard::diff_pressure_present()
{
  return airspeed_.present();
}

void AirbotBoard::diff_pressure_update()
{
  airspeed_.update();
}


void AirbotBoard::diff_pressure_read(float *diff_pressure, float *temperature)
{
  (void) diff_pressure;
  (void) temperature;
  airspeed_.update();
  airspeed_.read(diff_pressure, temperature);
}

bool AirbotBoard::sonar_present()
{
  return sonar_.present();
}

void AirbotBoard::sonar_update()
{
  sonar_.update();
}

float AirbotBoard::sonar_read()
{
  return sonar_.read();
}

bool AirbotBoard::gnss_present()
{
  return gnss_.present();
}
void AirbotBoard::gnss_update(){}
bool AirbotBoard::gnss_has_new_data()
{
  return this->gnss_.new_data();
}
GNSSData AirbotBoard::gnss_read()
{
  UBLOX::GNSSPVT gnss_pvt= gnss_.read();
  UBLOX::GNSSPosECEF pos_ecef = gnss_.read_pos_ecef();
  UBLOX::GNSSVelECEF vel_ecef = gnss_.read_vel_ecef();
  GNSSData gnss = {};
  gnss.time_of_week = gnss_pvt.time_of_week;
  gnss.time = gnss_pvt.time;
  gnss.nanos = gnss_pvt.nanos;
  gnss.lat = gnss_pvt.lat;
  gnss.lon = gnss_pvt.lon;
  gnss.height = gnss_pvt.height;
  gnss.vel_n = gnss_pvt.vel_n;
  gnss.vel_e = gnss_pvt.vel_e;
  gnss.vel_d = gnss_pvt.vel_d;
  gnss.h_acc = gnss_pvt.h_acc;
  gnss.v_acc = gnss_pvt.v_acc;
  gnss.ecef.x = pos_ecef.x;
  gnss.ecef.y = pos_ecef.y;
  gnss.ecef.z = pos_ecef.z;
  gnss.ecef.p_acc = pos_ecef.p_acc;
  gnss.ecef.vx = vel_ecef.vx;
  gnss.ecef.vy = vel_ecef.vy;
  gnss.ecef.vz = vel_ecef.vz;
  gnss.ecef.s_acc = vel_ecef.s_acc;

  return gnss;
}
GNSSRaw AirbotBoard::gnss_raw_read()
{	
  UBLOX::NAV_PVT_t pvt = gnss_.read_raw();
  GNSSRaw raw = {};
  raw.time_of_week = pvt.iTOW;
  raw.year = pvt.time.year;
  raw.month = pvt.time.month;
  raw.day = pvt.time.day;
  raw.hour = pvt.time.hour;
  raw.min = pvt.time.min;
  raw.sec = pvt.time.sec;
  raw.valid = pvt.time.valid;
  raw.t_acc = pvt.time.tAcc;
  raw.nano = pvt.time.nano;
  raw.fix_type = pvt.fixType;
  raw.num_sat = pvt.numSV;
  raw.lon = pvt.lon;
  raw.lat = pvt.lat;
  raw.height = pvt.height;
  raw.height_msl = pvt.hMSL;
  raw.h_acc = pvt.hAcc;
  raw.v_acc = pvt.vAcc;
  raw.vel_n = pvt.velN;
  raw.vel_e = pvt.velE;
  raw.vel_d = pvt.velD;
  raw.g_speed = pvt.gSpeed;
  raw.head_mot = pvt.headMot;
  raw.s_acc = pvt.sAcc;
  raw.head_acc = pvt.headAcc;
  raw.p_dop = pvt.pDOP;
  raw.rosflight_timestamp = gnss_.get_last_pvt_timestamp();
  return raw;
}

//MULTI_RANGE
bool AirbotBoard::multi_range_present()
{
	return multi_range_.present();
}

void AirbotBoard::multi_range_update()
{
	multi_range_.update();
}

bool AirbotBoard::multi_range_has_new_data()
{
	return multi_range_.has_new_data();
}
	
uint8_t AirbotBoard::multi_range_get_nb_sensors()
{
	return multi_range_.getNbSensors();
}

void AirbotBoard::multi_range_read(uint16_t *ranges)
{
	return multi_range_.read(ranges);
}

// PWM
void AirbotBoard::rc_init(rc_type_t rc_type)
{
	DBG("rc_init\n");
	
  switch (rc_type)
  {
  case RC_TYPE_SBUS:
    sbus_uart_.init(&uart_config[0], 100000, UART::MODE_8E2);
    inv_pin_.init(SBUS_INV_GPIO, SBUS_INV_PIN, GPIO::OUTPUT);
    rc_sbus_.init(&inv_pin_, &sbus_uart_);
    rc_ = &rc_sbus_;
    break;
  case RC_TYPE_PPM:
  default:
    rc_ppm_.init(&pwm_config[RC_PPM_PIN]);
    rc_ = &rc_ppm_;
    break;
  }
}

float AirbotBoard::rc_read(uint8_t channel)
{
  return rc_->read(channel);
}

void AirbotBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
	DBG("pwm_init\n");
	
  for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  {
    esc_out_[i].init(&pwm_config[i], refresh_rate, 2000, 1000);
    esc_out_[i].writeUs(idle_pwm);
  }
	
}

void AirbotBoard::pwm_disable()
{
	for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  {
    esc_out_[i].disable();
  }
}

void AirbotBoard::pwm_write(uint8_t channel, float value)
{
	esc_out_[channel].write(value);
}

bool AirbotBoard::rc_lost()
{
  return rc_->lost();
}

// non-volatile memory
void AirbotBoard::memory_init()
{
	DBG("memory_init\n");
  return flash_.init(&spi3_);
}

bool AirbotBoard::memory_read(void * data, size_t len)
{
  return flash_.read_config(reinterpret_cast<uint8_t*>(data), len);
}

bool AirbotBoard::memory_write(const void * data, size_t len)
{
  return flash_.write_config(reinterpret_cast<const uint8_t*>(data), len);
}

// LED
void AirbotBoard::led0_on() {}
void AirbotBoard::led0_off() {}
void AirbotBoard::led0_toggle() {}

void AirbotBoard::led1_on() {led2_.on();}
void AirbotBoard::led1_off() {led2_.off();}
void AirbotBoard::led1_toggle() {led2_.toggle();}

//BUZZER
void AirbotBoard::buzzer_on() { buzzer_.off(); }
void AirbotBoard::buzzer_off() { buzzer_.on(); }

bool AirbotBoard::battery_voltage_present() {return true;}
void AirbotBoard::battery_voltage_update()
{
	battery_voltage_ = BAT_VOLTAGE_MULT * voltage_input_.read();
}
	
float AirbotBoard::battery_voltage_read() {return battery_voltage_;}

//Backup memory
bool AirbotBoard::has_backup_data()
{
  BackupData backup_data = backup_sram_read();
  return (check_backup_checksum(backup_data) && backup_data.error_code!=0);
}
rosflight_firmware::BackupData AirbotBoard::get_backup_data()
{
  return backup_sram_read();
}

} // namespace rosflight_firmware

extern "C"
{
	void PPM_RC_IQRHandler(void)
	{
		RC_PPM::irq_handler();
	}
}
