#include <AccelStepper.h>
#include "ScaraArm.h"
#include "ScaraController.h"
// pins: STEP1,DIR1, STEP2,DIR2, STEP3,DIR3, STEPZ,DIRZ,
// EN, LIM1, LIM2, LIM3, LIMZ
ScaraArm arm(2,5, 3,6, 4,7, 12,13, 8,9, 11,10, A3);
ScaraController ctl(arm);
void toggleLED() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  arm.initLength(2280.0F, 1365.0F);
  arm.initStoD  (11.544444F, 8.755555F, 2.8F, 1.0F);
  ctl.begin(9600);
  ctl.registerAction('A', toggleLED);
  String seqB =
    "7,3,360,3470,17000,0,-420,3450,17000,0,290,3340,17000,0;9,3,440,3350,17000,0,-440,3380,17000,0,410,3440,17000,0";
  ctl.registerSequence('B', seqB);
}
void loop() {
  ctl.update();
  ctl.process();
}
