#include "BLESerial.h"
//#include <BLEPeripheral.h>
#include"utility/BLEConstants.h"
#include <Adafruit_STMPE610.h>
#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output

#define FACTORYRESET_ENABLE         1

// SOFTWARE UART SETTINGS
#define BLUEFRUIT_SWUART_RXD_PIN       9    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       10   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         8   // Optional, set to -1 if unused
#define BLUEFRUIT_UART_MODE_PIN        12    // Set to -1 if unused

// Create the bluefruit object, either software serial...uncomment these lines
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN, BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

//BLESerial ble(9, 10);

void setup() {
  // put your setup code here, to run once:
   // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  //pinMode(9, OUTPUT);
  ble.enableSerial(false); // Defaults to true but you can set it to false to disable any usage of the hardware serial
  ble.setBLEBaud(9600); // Set the Baud rate for software serial to connect with module, defaults to 9600
  ble.begin(); // Initialize the serial ports
  ble.setName("SPBLE") // Set BLE Device Name

  Serial.println(ble.getName()); // Read BLE Name

  ble.setBaud(BAUD_9600) // Set Baudrate based on BLEConstants.h

  Serial.println(ble.getBaud()); // Read Baudrate

  ble.setPin("000000"); // Set pairing pin

  Serial.println(ble.getPin()) //Read Pin
}

void loop() {

   
 // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.print(sensorValue);
  Serial.print(",");
  //the above is the same as below
  //Serial.println(sensorValue);
  //send it over bluetooth
  //BLESerial.print(sensorValue);
  //BLESerial.print(", ");
  
  char dat; //Variable to store received byte
  if (ble.available()) {
    dat = ble.read();
    Serial.print(dat); // Print all received data to Serial Console
  }

  if (Serial.available()) {
    dat = Serial.read();
    ble.write(dat); // Send any data received from Serial to ble device.
  }
  
  delay(100);        // delay in between reads for stability
}
