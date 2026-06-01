/*!
 * @file MHT7042A.h
 *
 * Designed specifically to work with the Winsen MH-T7042A sensor
 * ----> https://www.winsen-sensor.com/product/mh-t7042a.html
 *
 * These sensors communicate over UART and require two signal pins.
 * 
 * Written by Agustin Capovilla for the Research Institute for Signals, Systems
 *  and Computational Intelligence (sinc(i)) (UNL-CONICET)
 *
 * MIT license, all text here must be included in any redistribution.
 * See the LICENSE file for details.
 *
 */

#ifndef __MHT7042A_H__
#define __MHT7042A_H__

#include "Arduino.h"

#include <Adafruit_Sensor.h>

/**
 * @brief  Default sensor baudrate
 */
#define MHT7042A_DEFAULT_BAUDRATE (9600)          // Sensor baudrate

/**
 * @brief Default serial configuration (8 data bits, 1 stop bit, and no parity)
 */
#define MHT7042A_DEFAULT_SERIAL_CONFIG (SERIAL_8N1) // Sensor serial parameters

/**
 *  @brief  MH-T7042A sensor protocol packet length
 */
#define MHT7042A_PACKETLEN (9)            // Packet length

/**
 * @brief A class to interact with MHT7042A sensor
 *
 */
class MHT7042A : public Adafruit_Sensor {
public:
  MHT7042A(Stream *serial, int32_t sensorID = 1);
  ~MHT7042A();

  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);
  
private:
  int getGasConcentration(void);

  unsigned char getCheckSum(uint8_t *packet);

  void writeCommand(uint8_t command);
  
  void readPacket(uint8_t *buffer);
  void writePacket(uint8_t *buffer);
  int getAvailable(void);
  void flushInputBuffer(void);

  Stream *_serial_dev;
  int32_t _sensorID;
  uint8_t _buffer[MHT7042A_PACKETLEN];
};

#endif // __MHT7042A_H__