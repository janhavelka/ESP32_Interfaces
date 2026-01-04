// README snippet (Serial console examples):
//   mode gpio
//   gpio set 5
//   gpio toggle 100000
//   gpio pulse 5 20
//   mode uart
//   uart start 1 17 18 921600
//   uart pattern 55
//   mode jtag
//   jtag bitbang 39 40 41 42 10000

#include <Arduino.h>
#include "RuntimeProbe.h"

static RuntimeProbe gProbe;

void setup() { gProbe.begin(); }

void loop() { gProbe.update(); }
