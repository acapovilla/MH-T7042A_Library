/**
 * @file MHT7042A.cpp
 *
 * @mainpage MHT7042A Library for the Winsen MH-T7042A UART CH4 Gas Sensor
 *
 * @section intro_sec Introduction
 *
 * 	This is a library for the Winsen MH-T7042A infrared methane (CH4) gas
 *  sensor.
 * 	https://www.winsen-sensor.com/product/mh-t7042a.html
 *
 * @section dependencies Dependencies
 *  This library depends on the Adafruit Unified Sensor library
 *
 * @section author Author
 *
 *  Agustin Capovilla for Research institute for signals, systems and 
 * computational intelligence (sinc(i)) (UNL - CONICET)
 *
 * 	@section license License
 *
 * 	MIT (see LICENSE)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"

#include "MHT7042A.h"

/**
 * @brief Default start byte
 */
#define MHT7042A_STARTBYTE (0xFF)         // Start Byte

/**
 * @brief Read Gas Concentration command 
 */
#define MHT7042A_CMD_READ_GAS (0x86)           // Read Gas Concentration command

/**
 * @brief Calibrate zero point command 
 */
#define MHT7042A_CMD_ZERO_CAL (0x87)           // Calibrate zero point command

/**
 * @brief Calibrate span point command 
 */
#define MHT7042A_CMD_SPAN_CAL (0x88)           // Calibrate span point command

/**
 * @brief Timeout for sensor communication, in milliseconds.
 */
#define MHT7042A_TIMEOUT_MS 2000          //  Default timeout


/**
 * @brief Construct a new MHT7042A::MHT7042A object
 * @param serial Pointer to a Stream interface 
 *               (HardwareSerial or SoftwareSerial)
 * @param sensorID An optional ID that will be placed in sensor events to help
 *                 keep track if you have many sensors in use
 * 
 * @note This library does not initialize the serial interface.
 *       Call begin() on the serial port before using the sensor.
 */
MHT7042A::MHT7042A(Stream *serial, int32_t sensorID) 
  : _serial_dev(serial), _sensorID(sensorID) {}

/**
 * @brief Destroy the MHT7042A::MHT7042A object
 *
 */
MHT7042A::~MHT7042A(void) {}

/**
 * @brief Populates a sensor_t structure with sensor details
 * @param sensor A pointer to a sensor_t structure that we will fill with
 *                details about the MH-T7042A and its capabilities
 */
void MHT7042A::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MH-T7042A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_UNITLESS_PERCENT;
  sensor->min_delay = 0;        /*  zero = not a constant rate */
  sensor->min_value = 0.0;      /* %Vol range 0.0 ~ 100.0%  */
  sensor->max_value = +100.0;
  sensor->resolution = 0.1;     /*  0.1 %Vol */
}


/**
 * @brief Get the gas concentration reading from the sensor
 *
 * @param event Pointer to a sensor_event_t type that will be filled
 *              with the gas concentration reading, timestamp, data type and
 *              sensor ID
 * @return true if a valid reading was received, false otherwise.
 */
bool MHT7042A::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_UNITLESS_PERCENT;
  event->timestamp = millis();

  int concentration = getGasConcentration();
  if (concentration >= 0 && concentration <= 10000) {
    event->unitless_percent = concentration / 100.f;
    return true;
  }
  
  event->unitless_percent = 0.f;
  return false;
}

/**
 * @brief Returns the Gas Concentration Reading from the sensor
 * @returns Concentration in 0.01 %Vol units, or -1 on communication error.
 */
int MHT7042A::getGasConcentration(void) {
  // get next measurement
  writeCommand(MHT7042A_CMD_READ_GAS);

  int concentration = -1;

  // Store current time to measure the timeout
  uint32_t timeout_start = millis();

  // wait until response has been received, or the timeout occurred after 
  // MHT7042A_TIMEOUT_MS ms.
  while (getAvailable() < (MHT7042A_PACKETLEN + 1)) {
    // In case of a timeout, stop the while loop
    if ((millis() - timeout_start) > MHT7042A_TIMEOUT_MS) {
      return concentration;
    }
    delay(1);
  }

  readPacket(_buffer);

  if (_buffer[0] == MHT7042A_STARTBYTE) {
      if (_buffer[1] == MHT7042A_CMD_READ_GAS) {
        unsigned char checksum = getCheckSum(_buffer);

        if (_buffer[MHT7042A_PACKETLEN-1] == checksum) {
          concentration = _buffer[2];
          concentration = (concentration << 8) + _buffer[3];

          return concentration;
        } else {
          // Bad Check Sum
          flushInputBuffer();
        }
      } else {
        // Bad Command byte
        flushInputBuffer();
      }
  } else {
    // Bad Start byte
    flushInputBuffer();
  }

  return concentration;
}

/**
 * @brief Sends a command packet to the sensor.
 * @param command Command byte to transmit.
 */
void MHT7042A::writeCommand(uint8_t command) {
    _buffer[0] = MHT7042A_STARTBYTE;
    _buffer[1] = 0x01;
    _buffer[2] = command;
    uint8_t crc = getCheckSum(_buffer);
    _buffer[MHT7042A_PACKETLEN-1] = crc;
    writePacket(_buffer);
    return;
}

/**
 * @brief Computes the checksum for a sensor packet.
 * @param packet Pointer to the packet buffer.
 * @return Calculated checksum value.
 */
unsigned char MHT7042A::getCheckSum(uint8_t *packet) {
  unsigned char i, checksum = 0x00;
  
  for (i = 1; i < (MHT7042A_PACKETLEN-1); i++) {
    checksum += packet[i];
  }

  checksum = 0xFF - checksum;
  checksum += 1;
  return checksum;
}

/**
 * @brief Reads a packet from the serial interface.
 * @param buffer Buffer where the received packet is stored.
 */
void MHT7042A::readPacket(uint8_t *buffer) {
  memset(buffer, 0, MHT7042A_PACKETLEN);
  (*_serial_dev).readBytes(buffer, MHT7042A_PACKETLEN);
}

/**
 * @brief Writes a packet to the serial interface.
 * @param buffer Packet buffer to transmit.
 */
void MHT7042A::writePacket(uint8_t *buffer) {
  (*_serial_dev).write(buffer, MHT7042A_PACKETLEN);
}

/**
 * @brief Returns the number of bytes available in the serial buffer.
 * @return Number of bytes available to read.
 */
int MHT7042A::getAvailable(void) {
  return (*_serial_dev).available();
}


/**
 * @brief Discards one byte from the serial receive buffer.
 */
void MHT7042A::flushInputBuffer(void) {
  while ((*_serial_dev).available()) {
    (*_serial_dev).read();
  }
}