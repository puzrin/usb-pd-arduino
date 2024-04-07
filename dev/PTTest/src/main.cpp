//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>
#include <Wire.h>
#include "I2C.h"
#include "Timers.h"


class BackgroundTask : public Continuation
{
public:
  bool readDeviceId = false;

protected:
  bool ledOn = true;
  uint8_t deviceId;

  virtual void run();
};

BackgroundTask backgroundTask{};
HardwareTimer hardwareTimer(TIM1);
Timer blinkDelay(&backgroundTask);
TwoWire Wire3(PB4, PA7);
I2C i2c(&Wire3, &backgroundTask);


void onButtonChanged() {
  // read device ID when button is pressed
  bool buttonPressed = digitalRead(A0) == LOW;
  if (buttonPressed) {
    backgroundTask.readDeviceId = true;
    backgroundTask.notifyFromInterrupt();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello World");
  pinMode(LED_BUILTIN, OUTPUT);

  Scheduler::setHardwareTimer(&hardwareTimer);
  Scheduler::start();

  pinMode(A0, INPUT);
  attachInterrupt(digitalPinToInterrupt(A0), onButtonChanged, CHANGE);

  Wire3.setClock(1000000);
  Wire3.begin();

  backgroundTask.start();
}

void loop() {
  // read device ID every 5 seconds
  delay(5000);
  backgroundTask.readDeviceId = true;
  backgroundTask.notifyFromApp();
}

void BackgroundTask::run()
{
  CT_BEGIN

    // blink LED and read device ID (when triggered)
    while (true)
    {      
      digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
      blinkDelay.scheduleIn(1000);

      do {
        CT_WAIT_UNTIL(blinkDelay.hasExpired() || readDeviceId);
        if (readDeviceId) {
          i2c.startReadRegister(0x22, 0x01, &deviceId);
          CT_WAIT_I2C(&i2c);
          readDeviceId = false;
        }
      } while (blinkDelay.isRunning());

      ledOn = !ledOn;
    }

  CT_END
}
