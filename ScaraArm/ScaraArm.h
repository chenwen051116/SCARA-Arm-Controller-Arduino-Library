// ScaraArm.h
#ifndef SCARAARM_H
#define SCARAARM_H

#include <AccelStepper.h>
#include <Arduino.h>

class ScaraArm {
public:
  // constructor: pins for joint1, joint2, joint3, Z, enable, and limit switches
  ScaraArm(byte s1Step, byte s1Dir,
           byte s2Step, byte s2Dir,
           byte s3Step, byte s3Dir,
           byte zStep,  byte zDir,
           byte enPin,
           byte limit1Pin, byte limit2Pin, byte limit3Pin, byte limitZPin);

  // configure link lengths (mm)
  void initLength(float L1, float L2);
  // configure steps-per-unit: θ1, θ2, θ3 (steps/deg), Z (steps/unit)
  void initStoD(float sto1, float sto2, float sto3, float stoZ);

  // call in setup()
  void begin(long baud = 9600);
  // call in loop()
  void update();

  // move joints 1,2,3 by angles (deg) and Z by steps
  void runtoPos(int theta1, int theta2, int theta3, int zSteps);
  // move in XY (mm), Z (steps), and set joint3 angle
  bool runtoPosxy(int x, int y, int zSteps, int theta3);
  // move joint3 only
  void runtoJoint3(int theta3);

  // set speed & acceleration
  void setVel(int speed, int accel);
  // block until within tolerance
  void waitForMotionComplete();

  // execute an array of [x,y,z,theta3]
  void executePath(int path[][4], int count);

  // smoothly pass through a series of [x,y,z,θ3] waypoints without stopping
  // threshold = how close (in steps) before loading the next target
  bool runThroughPath(int path[][4], int count, long thresholdSteps = 10);

  // home all axes
  void homing();
  // clear serial buffer
  void serialFlush();
  // get current Z
  long getCurrentZ();
// returns joint3 angle in degrees
  long getJoint3Angle();


  enum ActionType { MOVE_POS, MOVE_XY, MOVE_JOINT3, CUSTOM_FUNC };
  using Callback = void(*)(ScaraArm&);

  struct Action {
    ActionType type;
    int a, b, c, d;
    Callback fn;
  };

  // clear any queued actions
  void clearActions();
  // queue up one of the built-in moves
  bool addMoveAction(int theta1, int theta2, int theta3, int zSteps);
  bool addXYZAction(int x, int y, int zSteps, int theta3);
  bool addJoint3Action(int theta3);
  // queue up an arbitrary function
  bool addCustomAction(Callback fn);
  // run all queued actions in order (blocking)
  void executeSeries();

  /*
  example use of Series: 
    // queue up a little series…
  arm.clearActions();
  arm.addMoveAction(  0,   0,   0, 17000);
  arm.addXYAction( 100, 200, 17000,   0);
  arm.addJoint3Action( 45 );
  arm.addCustomAction([](ScaraArm &a){
    // any custom code you want the arm to do!
    // e.g. blink an LED or print status:
    Serial.println("custom step reached");
  });
  arm.executeSeries();  // runs each queued step in sequence
  */

private:
  AccelStepper* stepper1;
  AccelStepper* stepper2;
  AccelStepper* stepper3;
  AccelStepper* stepperZ;
  byte _enPin, _lim1, _lim2, _lim3, _limZ;
  long stepperPos[4];
  double angle[2];

  float _L1, _L2;
  float _sto1, _sto2, _sto3, _stoZ;
  const float _rad2deg = 180.0F / 3.1415926F;

  float _D1_min = 190, _D1_max = -190;
  float _D2_min = 190, _D2_max = -190;
  const long _tol = 1;

  bool solveAng(int xx, int yy);

  static const int MAX_ACTIONS = 20;
  Action _actions[MAX_ACTIONS];
  int _actionCount = 0;

  // helper used by executeSeries()
  void _runAction(const Action& act);
};

#endif