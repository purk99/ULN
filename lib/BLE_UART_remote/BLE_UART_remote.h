
// supported build flags
// -D UART_WITH_SERIAL_FEEDBACK -> lib will write some output to Serial
// -D UART_WITH_LED_FEEDBACK -> lib will blink the onboard LED when commands received
// -D UART_WITH_REMOTE_FEEDBACK -> lib will mirror the received command to BLE UART sender
//

#ifndef _BLE_UART_REMOTE_H_
#define _BLE_UART_REMOTE_H_

bool bleUartSetup(char* bleName);
void bleUartLoop();
void bleUartCommandHandled();

extern char command;

#endif