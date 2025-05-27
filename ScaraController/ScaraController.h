// ScaraController.h

#ifndef SCARACONTROLLER_H
#define SCARACONTROLLER_H

#include <Arduino.h>
#include "ScaraArm.h"

// callback type for A–K
typedef void (*ActionFn)();

class ScaraController {
public:
  // ctor takes your ScaraArm
  ScaraController(ScaraArm &arm);

  // bind a simple void() callback to 'A'…'K'
  void registerAction(char key, ActionFn fn);
  // bind a semicolon-separated command sequence to 'A'…'K'
  void registerSequence(char key, const String &seq);

  // initialize Serial + arm (prints "finished initial")
  void begin(unsigned long baud = 9600);

  // to be called in loop()
  void update();   // run steppers
  void process();  // read & dispatch commands

private:
  ScaraArm &arm_;
  ActionFn  charActions_[11];
  String    charSequences_[11];

  void handleCommand(const String &line);
  void executeSingle(const String &line);
  void runSequence(const String &seq);
  void flushInput();
};

#endif
