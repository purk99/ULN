/*
 see https://forum.arduino.cc/t/uart-serial-port-emulation-over-ble-protocol/885285
 and https://github.com/Uberi/Arduino-HardwareBLESerial
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <HardwareBLESerial.h>

#include "BLE_UART_remote.h"

HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();

char command = 0x00;

bool bleUartSetup(char* bleName)
{
    if (!bleSerial.beginAndSetupBLE(bleName))
    {
#ifdef UART_WITH_SERIAL_FEEDBACK
        Serial.print("BLE UART device failed to initialize.");
#endif
        return false;
    }

#ifdef UART_WITH_LED_FEEDBACK
    pinMode(LED_BUILTIN, OUTPUT);
#endif
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.print("BLE UART device available");
#endif

    return true;
}

void bleUartLoop()
{
    // this must be called regularly to perform BLE updates
    bleSerial.poll();

    while (bleSerial.availableLines() > 0) {

#ifdef UART_WITH_LED_FEEDBACK
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
#endif
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.print("I received: ");
#endif
#ifdef UART_WITH_REMOTE_FEEDBACK
    bleSerial.print("I received: ");
#endif

    char uartCmd[5]; bleSerial.readLine(uartCmd, 5);
    switch (uartCmd[0]) {

      case '.':
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.println("NO OP - NO OP");
#endif
#ifdef UART_WITH_REMOTE_FEEDBACK
    bleSerial.println("NO OP - NO OP");
#endif
        command = uartCmd[0];
        break;

      case 'F':
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.println("VORWÄRTS");
#endif
#ifdef UART_WITH_REMOTE_FEEDBACK
    bleSerial.println("VORWÄRTS");
#endif
        command = uartCmd[0];
        break;

      case 'B':
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.println("VORWÄRTS");
#endif
#ifdef UART_WITH_REMOTE_FEEDBACK
    bleSerial.println("VORWÄRTS");
#endif
        command = uartCmd[0];
        break;

      case 'L':
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.println("LINKS");
#endif
#ifdef UART_WITH_REMOTE_FEEDBACK
    bleSerial.println("LINKS");
#endif
        command = uartCmd[0];
        break;

      case 'R':
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.println("RECHTS");
#endif
#ifdef UART_WITH_REMOTE_FEEDBACK
    bleSerial.println("RECHTS");
#endif
        command = uartCmd[0];
        break;

      case 'D':
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.println("DANCE DANCE DANCE");
#endif
#ifdef UART_WITH_REMOTE_FEEDBACK
    bleSerial.println("DANCE DANCE DANCE");
#endif
        command = uartCmd[0];
        break;

      default:
#ifdef UART_WITH_SERIAL_FEEDBACK
    Serial.println("... something I don't understand ;-)");
#endif
#ifdef UART_WITH_REMOTE_FEEDBACK
    bleSerial.println("... something I don't understand ;-)");
#endif
        command = 0x00;
    }
  }
}

void bleUartCommandHandled() {
    command = 0x00;
}
