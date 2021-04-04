/*

  TruStability HSC and SSC pressure sensor library for the Arduino.

  This library implements the following features:

  - read raw pressure and temperature count values
  - compute absolute pressure and temperature

  Author:          Petre Rodan <petre.rodan@simplex.ro>
  Available from:  https://github.com/rodan/honeywell_hsc_ssc_i2c
  License:         GNU GPLv3

  Honeywell High Accuracy Ceramic (HSC) and Standard Accuracy Ceramic
  (SSC) Series are piezoresistive silicon pressure sensors.


  GNU GPLv3 license:
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include "hsc_ssc_i2c.h"

#include <Wire.h>

#define OUTPUT_MIN   0x666   // 10%
#define OUTPUT_MAX   0x3999  // 90% of 2^14 - 1
#define PRESSURE_MIN 0.0     // min is 0 for sensors that give absolute values
#define PRESSURE_MAX 60.0    // Sensor HSCMAND060PA3A3: 60psi

/* you must define the slave address. you can find it based on the part number:

    _SC_________XA_
    where X can be one of:

    S  - spi (this is not the right library for spi opperation)
    2  - i2c slave address 0x28
    3  - i2c slave address 0x38
    4  - i2c slave address 0x48
    5  - i2c slave address 0x58
    6  - i2c slave address 0x68
    7  - i2c slave address 0x78
*/

/*
  -----------------
  Requests raw data from the HSC pressure sensor via i2c
  -----------------

  input
    slave_addr    - i2c slave addr of the sensor chip
  output
    raw           - struct containing 4 bytes of read data

returns
        0 if all is fine
        1 if chip is in command mode
        2 if old data is being read
        3 if a diagnostic fault is triggered in the chip
        4 if the sensor is not hooked up
*/
uint8_t ps_get_raw(const uint8_t slave_addr, struct cs_raw *raw) {
  uint8_t i, val[4] = {0, 0, 0, 0};

  Wire.requestFrom(slave_addr, (uint8_t)4);

  for (i = 0; i <= 3; i++) {
    delay(4);              // sensor might be missing, do not block
    val[i] = Wire.read();  // by using Wire.available()
  }
  raw->status = (val[0] & 0xC0) >> 6;  // first 2 bits from first byte
  raw->bridge_data = ((val[0] & 0x3F) << 8) + val[1];
  raw->temperature_data = ((val[2] << 8) + (val[3] & 0xE0)) >> 5;
  if (raw->temperature_data == 65535) return 4;
  return raw->status;
}

/*
  -----------------
  Converts raw data read from the HSC sensor into temperature and pressure values
  -----------------

  input:
    raw            - struct containing all 4 bytes read from the sensor
    output_min     - output at minimal calibrated pressure (counts)
    output_max     - output at maximum calibrated pressure (counts)
    pressure_min   - minimal value of pressure range
    pressure_max   - maxium value of pressure range

  output:
    pressure
    temperature
*/
uint8_t ps_convert(const struct cs_raw raw, float *pressure, float *temperature) {
  const uint16_t output_min = OUTPUT_MIN;
  const uint16_t output_max = OUTPUT_MAX;
  const float pressure_min = PRESSURE_MIN;
  const float pressure_max = PRESSURE_MAX;

  // Calculate absolut pressure
  // Check we don't have a null pointer
  if (pressure) {
    *pressure = ((raw.bridge_data - output_min) * (pressure_max - pressure_min)) / (output_max - output_min) + pressure_min;
  }

  // Calculate temperature
  // Check we don't have a null pointer
  if (temperature) {  // Check we don't have a null pointer
    *temperature = ((float)raw.temperature_data * 0.0977) - 50.0;
  }

  return 0;
}

/*
  -----------------
  Read HSC pressure sensor absolute pressure and temperature
  -----------------
*/
uint8_t read_hsc_absolute(const uint8_t slave_addr, float *abs_press, float *temperature) {
  struct cs_raw hsc_raw;
  uint8_t status;

  // Read the raw HSC absolute pressure and temperature values
  status = ps_get_raw(slave_addr, &hsc_raw);

  // Check the I2C read status from the pressure sensor
  if (!status) {
    // Convert the raw pressure sensor bridge data to a pressure and temperature
    ps_convert(hsc_raw, abs_press, temperature);
  } else {
    if (abs_press)  // Check we don't have a null pointer
      *abs_press = 0.0;

    if (temperature)  // Check we don't have a null pointer
      *temperature = 0.0;
    print_hsc_error(status);
  }

  return status;
}

/*
  -----------------
  Read HSC gauge pressure
  -----------------
*/
float read_hsc_gauge(const uint8_t slave_addr, float atmos) {
  uint8_t status;
  float abs_p;
  float gauge;

  // Read absolute pressure
  status = read_hsc_absolute(slave_addr, &abs_p, nullptr);

  if (!status) {
    // Convert absolute pressure to gauge pressure
    gauge = abs_p - atmos;
    if (gauge < 0.0) gauge = 0.0;
    log_i("Abs=%.2f psi, Atmos=%.2f psi, Gauge=%.2f psi\n", abs_p, atmos, gauge);
    return gauge;
  } else
    return 0;
}

/*
  -----------------
  Print HSC pressure sensor error
  -----------------
*/
void print_hsc_error(uint8_t status) {
  switch (status) {
    case 0:
      Serial.println("Good data!");
      break;

    case 1:
      Serial.println("Warning: HSC Pressure sensor command mode");
      break;

    case 2:
      Serial.println("warn stale HSC Pressure sensor data");
      break;

    case 3:
      Serial.println("Error:  HSC Pressure sensor diagnostic fault");
      break;

    case 4:
      Serial.println("Error: HSC Pressure sensor missing");
      break;

    default:
      Serial.printf("Error code %d is not recognised", status);
      break;
  }
}