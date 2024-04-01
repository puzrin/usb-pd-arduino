# USB Power Delivery for Arduino

Implement a USB PD trigger board or a more sophisticated power sink using simple Arduino code. Supports STM32 microcontrollers with UCPD peripheral and ESP32 microcontrollers with an external FUSB302 PD controller.



## Supported Boards

| Board | Required additional components |
| - | - |
| Nucleo-G071RB | None |
| Nucleo-G431KB | None |
| Nucleo-G474RE | None |
| ESP32 | FUSB302 USB PD controller |

All boards require an additional USB C connector as the standard connector is not ready for USB Power Delivery (no USB C connector, CC1/CC2 signals not available, voltage regular cannot handle more than 5V).
For the Nucelo boards in Nucleo-64 form factor, the X-NUCLEO-SNK1M1 shield can be used.

See the Wiki for how to wire the board and the additional components.


## Library Installation (Arduino IDE)

1. In the Arduino IDE, navigate to *Sketch > Include Library > Manage Libraries...*

2. The Library Manager will open and you will find a list of libraries that are already installed or ready for installation.

3. Search for *Power Delivery* using the search bar.

4. Click on the *INSTALL* button to install it.



## Examples

### Trigger Board

The trigger boards communicates with a USB power supply and requests a different voltage than the initial 5V.

```c++
#include "USBPowerDelivery.h"

void setup() {
  PowerSink.start();
  // request 12V @ 1A once power supply is connected
  PowerSink.requestPower(12000, 1000);
}

void loop() {
  // nothing to do
}
```

See the Wiki for details regarding the required components and wiring.


## Restrictions

- This library uses several peripherals exclusively (e.g. one or two timers and the ADC in many cases). The peripheral can no longer be used by your own code as it would interfer with the operation of this library. Please read to restrictions that apply to your board.

- The USB PD protocol is very timing sensitive. In order to be robust even in the presence of blocking code (e.g. `Serial.println()`), most of the USB PD processing is done in interrupt handlers. While receiving a USB PD message, interrupt handlers can consume up to 40% of the CPU time.

