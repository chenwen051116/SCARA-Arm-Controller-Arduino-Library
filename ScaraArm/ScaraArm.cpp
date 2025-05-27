
// ScaraArm.cpp
#include "ScaraArm.h"

ScaraArm::ScaraArm(byte s1Step, byte s1Dir,
                   byte s2Step, byte s2Dir,
                   byte s3Step, byte s3Dir,
                   byte zStep,  byte zDir,
                   byte enPin,
                   byte limit1Pin, byte limit2Pin, byte limit3Pin, byte limitZPin)
  : _enPin(enPin), _lim1(limit1Pin), _lim2(limit2Pin), _lim3(limit3Pin), _limZ(limitZPin)
{
  stepper1 = new AccelStepper(AccelStepper::DRIVER, s1Step, s1Dir);
  stepper2 = new AccelStepper(AccelStepper::DRIVER, s2Step, s2Dir);
  stepper3 = new AccelStepper(AccelStepper::DRIVER, s3Step, s3Dir);
  stepperZ = new AccelStepper(AccelStepper::DRIVER, zStep,  zDir);
  _L1 = _L2 = 0;
  _sto1 = _sto2 = _sto3 = _stoZ = 1;
}

void ScaraArm::initLength(float L1, float L2) {
  _L1 = L1;
  _L2 = L2;
}

void ScaraArm::initStoD(float sto1, float sto2, float sto3, float stoZ) {
  _sto1 = sto1;
  _sto2 = sto2;
  _sto3 = sto3;
  _stoZ = stoZ;
}

void ScaraArm::begin(long baud) {
  Serial.begin(baud);
  pinMode(_enPin, OUTPUT);
  digitalWrite(_enPin, LOW);
  pinMode(_lim1, INPUT_PULLUP);
  pinMode(_lim2, INPUT_PULLUP);
  pinMode(_lim3, INPUT_PULLUP);
  pinMode(_limZ, INPUT_PULLUP);
  stepper1->setMaxSpeed(2000);  stepper1->setAcceleration(1500);
  stepper2->setMaxSpeed(1000);  stepper2->setAcceleration(1500);
  stepper3->setMaxSpeed(1500);  stepper3->setAcceleration(1500);
  stepperZ->setMaxSpeed(1500);  stepperZ->setAcceleration(1500);
  homing();
  setVel(300,300);
}

void ScaraArm::update() {
  stepper1->run();
  stepper2->run();
  stepper3->run();
  stepperZ->run();
}


void ScaraArm::runtoPos(int t1, int t2, int t3_abs, int z) {
  // convert absolute theta3 into the relative motor angle
  int t3_rel = t3_abs - (t1 + t2);                  // ← ABS→REL
  stepperPos[0] = long(t1    * _sto1);
  stepperPos[1] = long(t2    * _sto2);
  stepperPos[2] = long(t3_rel * _sto3);             // ← use relative
  stepperPos[3] = long(z     * _stoZ);

  stepper1->moveTo(stepperPos[0]);
  stepper2->moveTo(stepperPos[1]);
  stepper3->moveTo(stepperPos[2]);
  stepperZ->moveTo(stepperPos[3]);
}

bool ScaraArm::runtoPosxy(int x, int y, int z, int theta3) {
  if (solveAng(-x, y)) {
    float theta1 = angle[0], theta2 = angle[1];
    float t3_rel = theta3 - (theta1 + theta2);            // ← ABS→REL
    stepperPos[0] = long(angle[0] * _sto1);
    stepperPos[1] = long(angle[1] * _sto2);
    stepperPos[2] = long(t3_rel * _sto3);
    stepperPos[3] = long(z      * _stoZ);
    stepper1->moveTo(stepperPos[0]);
    stepper2->moveTo(stepperPos[1]);
    stepper3->moveTo(stepperPos[2]);
    stepperZ->moveTo(stepperPos[3]);
  }
  else{
    return false;
  }
  return true;
}

void ScaraArm::runtoJoint3(int t3) {
  float t3_rel = t3 - (angle[0] + angle[1]); // ← ABS→REL
  stepperPos[2] = long(t3_rel * _sto3);              // ← use relative
  stepper3->moveTo(stepperPos[2]);
}

void ScaraArm::setVel(int speed, int accel) {
  stepper1->setSpeed(speed);  stepper1->setAcceleration(accel);
  stepper2->setSpeed(speed);  stepper2->setAcceleration(accel);
  stepper3->setSpeed(speed);  stepper3->setAcceleration(accel);
  stepperZ->setSpeed(speed);  stepperZ->setAcceleration(accel);
}

void ScaraArm::waitForMotionComplete() {
  while (abs(stepper1->distanceToGo()) > _tol ||
         abs(stepper2->distanceToGo()) > _tol ||
         abs(stepper3->distanceToGo()) > _tol ||
         abs(stepperZ->distanceToGo()) > _tol) {
    update();
  }
}

void ScaraArm::executePath(int path[][4], int count) {
  for (int i = 0; i < count; i++) {
    if (!runtoPosxy(path[i][0], path[i][1], path[i][2], path[i][3])) {
      // unreachable → stop all motion immediately
      // (you can also call stepperX->stop() here if you want)
      return;
    }
    waitForMotionComplete();
  }
  return;
}

bool ScaraArm::runThroughPath(int path[][4], int count, long thresholdSteps) {
  if (count <= 0) return false;

  // helper to load a waypoint into the steppers
  auto loadPoint = [&](int idx){
    int x = path[idx][0], y = path[idx][1];
    int z = path[idx][2], th3 = path[idx][3];
    // solve for theta1,theta2
    if (!solveAng(-x, y)) return false;
    // convert to steps
    stepperPos[0] = long(angle[0] * _sto1);
    stepperPos[1] = long(angle[1] * _sto2);
    stepperPos[2] = long(th3      * _sto3);
    stepperPos[3] = long(z        * _stoZ);
    // issue new targets
    stepper1->moveTo(stepperPos[0]);
    stepper2->moveTo(stepperPos[1]);
    stepper3->moveTo(stepperPos[2]);
    stepperZ->moveTo(stepperPos[3]);
    return true;
  };

  // load first point
  if (!loadPoint(0)) return false;

  int idx = 0;
  // keep running until we reach last point
  while (true) {
    // keep motors ticking
    update();

    // check if we're within threshold of current target
    if (  abs(stepper1->distanceToGo()) < thresholdSteps
       && abs(stepper2->distanceToGo()) < thresholdSteps
       && abs(stepper3->distanceToGo()) < thresholdSteps
       && abs(stepperZ->distanceToGo()) < thresholdSteps ) {

      // advance to next waypoint
      idx++;
      if (idx >= count) break;            // done!
      if (!loadPoint(idx)) return false;  // failed to solve or load
    }
  }

  // optionally ensure we settle exactly at final
  waitForMotionComplete();
  return true;
}

bool ScaraArm::solveAng(int xx, int yy) {
  double x = xx, y = yy;
  float r = sqrt(x*x + y*y);
  if (r > (_L1 + _L2) || r < fabs(_L1 - _L2)) return false;
  float cD = (r*r - _L1*_L1 - _L2*_L2) / (2.0f * _L1 * _L2);
  cD = constrain(cD, -1.0f, 1.0f);
  float Delta = acos(cD);
  float cG = (_L1*_L1 + r*r - _L2*_L2) / (2.0f * _L1 * r);
  cG = constrain(cG, -1.0f, 1.0f);
  float Gamma = acos(cG);
  float Sigma = acos(y / r);
  if (x < 0.0f) Sigma = -Sigma;
  float D1a = (Sigma - Gamma) * _rad2deg;
  float D2a =  Delta         * _rad2deg;
  if (D1a < _D1_min && D1a > _D1_max && D2a < _D2_min && D2a > _D2_max) {
    angle[0] = D1a; angle[1] = D2a; return true;
  }
  float D1b = (Sigma + Gamma) * _rad2deg;
  float D2b = -Delta         * _rad2deg;
  if (D1b < _D1_min && D1b > _D1_max && D2b < _D2_min && D2b > _D2_max) {
    angle[0] = D1b; angle[1] = D2b; return true;
  }
  return false;
}


void ScaraArm::homing() {
  // Homing Stepper4
  // while (analogRead(_limZ) <= 200) {
  //   stepperZ->setSpeed(400);
  //   stepperZ->runSpeed();
      stepperZ->setCurrentPosition(17000);  
  // }
  // delay(20);
  // stepperZ->moveTo(17000);
  // while (stepperZ->currentPosition() != 17000) {
  //   stepperZ->run();
  // }

  // Homing Stepper3 (disabled in your original)
  // while (digitalRead(_lim3) != HIGH) {
  //   stepper3->setSpeed(-110);
  //   stepper3->runSpeed();
     stepper3->setCurrentPosition(0);
  // }
  // delay(20);
  // stepper3->moveTo(-1000);
  // while (stepper3->currentPosition() != -1000) {
  //   stepper3->run();
  // }

  // Homing Stepper2
  // while (digitalRead(_lim2) != HIGH) {
  //   stepper2->setSpeed(-500);
  //   stepper2->runSpeed();
  //  stepper2->setCurrentPosition(0);  // When limit switch pressed set position to 0 steps
    stepper2->setCurrentPosition(180*_sto2); 
  // }
  // delay(20);
  // stepper2->moveTo(1340);
  // while (stepper2->currentPosition() != 1340) {
  //   stepper2->run();
  // }
  // stepper2->setCurrentPosition(0);

  // Homing Stepper1
  // while (digitalRead(_lim1) != HIGH) {
  //   stepper1->setSpeed(-400);
  //   stepper1->runSpeed();
  //   stepper1->setCurrentPosition(0);  // When limit switch pressed set position to 0 steps
  // }
  // delay(20);
  // stepper1->moveTo(1290);
  // while (stepper1->currentPosition() != 1290) {
  //   stepper1->run();
  // }
  stepper1->setCurrentPosition(0);
}


void ScaraArm::serialFlush() {
  while (Serial.available() > 0) Serial.read();
}

long ScaraArm::getCurrentZ() {
  return stepperPos[3];
}


long ScaraArm::getJoint3Angle() {
  return stepperPos[2];
}

// clear queue
void ScaraArm::clearActions() {
  _actionCount = 0;
}

// add a move-to-angles action
bool ScaraArm::addMoveAction(int t1, int t2, int t3, int z) {
  if (_actionCount >= MAX_ACTIONS) return false;
  _actions[_actionCount++] = { MOVE_POS, t1, t2, t3, z, nullptr };
  return true;
}

// add an XY move (with Z & theta3)
bool ScaraArm::addXYZAction(int x, int y, int z, int th3) {
  if (_actionCount >= MAX_ACTIONS) return false;
  _actions[_actionCount++] = { MOVE_XY, x, y, z, th3, nullptr };
  return true;
}

// add only joint3 rotation
bool ScaraArm::addJoint3Action(int th3) {
  if (_actionCount >= MAX_ACTIONS) return false;
  _actions[_actionCount++] = { MOVE_JOINT3, th3, 0, 0, 0, nullptr };
  return true;
}

// add a custom callback
bool ScaraArm::addCustomAction(Callback fn) {
  if (_actionCount >= MAX_ACTIONS || fn == nullptr) return false;
  _actions[_actionCount++] = { CUSTOM_FUNC, 0,0,0,0, fn };
  return true;
}

// execute all queued actions in order
void ScaraArm::executeSeries() {
  for (int i = 0; i < _actionCount; ++i) {
    _runAction(_actions[i]);
    waitForMotionComplete();          // block until each finishes
  }
  clearActions();
}

// internal dispatcher
void ScaraArm::_runAction(const Action& act) {
  switch (act.type) {
    case MOVE_POS:
      runtoPos(act.a, act.b, act.c, act.d);
      break;
    case MOVE_XY:
      runtoPosxy(act.a, act.b, act.c, act.d);
      break;
    case MOVE_JOINT3:
      runtoJoint3(act.a);
      break;
    case CUSTOM_FUNC:
      act.fn(*this);
      break;
  }
}