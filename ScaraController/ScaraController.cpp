// ScaraController.cpp

#include "ScaraController.h"

ScaraController::ScaraController(ScaraArm &arm)
  : arm_(arm)
{
  for (int i = 0; i < 11; i++) {
    charActions_[i]   = nullptr;
    charSequences_[i] = "";
  }
}

void ScaraController::registerAction(char key, ActionFn fn) {
  if (key >= 'A' && key <= 'K') {
    charActions_[key - 'A'] = fn;
  }
}

void ScaraController::registerSequence(char key, const String &seq) {
  if (key >= 'A' && key <= 'K') {
    charSequences_[key - 'A'] = seq;
  }
}

void ScaraController::begin(unsigned long baud) {
  Serial.begin(baud);
  arm_.begin(baud);
  Serial.println("finished initial");
}

void ScaraController::update() {
  arm_.update();
}

void ScaraController::process() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length()) handleCommand(line);
}

void ScaraController::handleCommand(const String &line) {
  int idx = line.indexOf(',');
  int cmd = line.substring(0, idx).toInt();
  String content = line.substring(idx + 1);

  // helper to pull next int from content
  auto nextInt = [&](int &pos)->int {
    int c = content.indexOf(',', pos);
    String tok = c<0 ? content.substring(pos)
                    : content.substring(pos, c);
    int v = tok.toInt();
    pos = (c<0 ? content.length() : c+1);
    return v;
  };

  switch (cmd) {
    case 1: {
      int p[4], pos=0;
      for(int i=0;i<4;i++) p[i]=nextInt(pos);
      arm_.runtoPos(p[0],p[1],p[2],p[3]);
      arm_.waitForMotionComplete();
      break;
    }
    case 2: {
      idx = content.indexOf(',');
      arm_.setVel(content.substring(0,idx).toInt(),
                  content.substring(idx+1).toInt());
      break;
    }
    case 3: {
      int pos=0, x=nextInt(pos), y=nextInt(pos), z=nextInt(pos);
      arm_.runtoPosxy(x,y,z,0);
      arm_.waitForMotionComplete();
      break;
    }
    case 4: {
      idx = content.indexOf(',');
      int x=content.substring(0,idx).toInt();
      int y=content.substring(idx+1).toInt();
      arm_.runtoPosxy(x,y,arm_.getCurrentZ(),0);
      arm_.waitForMotionComplete();
      break;
    }
    case 5: {
      int pos=0, len=nextInt(pos);
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=arm_.getCurrentZ();
        path[i][3]=arm_.getJoint3Angle();
      }
      arm_.executePath(path,len);
      break;
    }
    case 6: {
      int pos=0, len=nextInt(pos);
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][3]=arm_.getJoint3Angle();
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=nextInt(pos);
      }
      arm_.executePath(path,len);
      break;
    }
    case 7: {
      int pos=0, len=nextInt(pos);
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=nextInt(pos);
        path[i][3]=nextInt(pos);
      }
      arm_.executePath(path,len);
      break;
    }
    case 8: {
      int pos=0, len=nextInt(pos);
      long th3 = arm_.getJoint3Angle();
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=nextInt(pos);
        path[i][3]=th3;
      }
      arm_.runThroughPath(path,len,20);
      break;
    }
    case 9: {
      int pos=0, len=nextInt(pos);
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=nextInt(pos);
        path[i][3]=nextInt(pos);
      }
      arm_.runThroughPath(path,len,20);
      break;
    }
    case 10: {
      char key = content.charAt(0);
      int i = key - 'A';
      if (i>=0 && i<11) {
        if (charActions_[i]) {
          charActions_[i]();
        }
        else if (charSequences_[i].length()) {
          runSequence(charSequences_[i]);
          return;  // runSequence prints its own final "done"
        }
      }
      break;
    }
  }

  Serial.println("done");
  arm_.serialFlush();
  flushInput();
}

void ScaraController::executeSingle(const String &line) {
  // same parsing for one command, but no Serial.println
  int idx = line.indexOf(',');
  int cmd = line.substring(0, idx).toInt();
  String content = line.substring(idx + 1);
  auto nextInt = [&](int &pos)->int {
    int c=content.indexOf(',',pos);
    String t=c<0?content.substring(pos)
               :content.substring(pos,c);
    int v=t.toInt();
    pos=(c<0?content.length():c+1);
    return v;
  };

  switch (cmd) {
    case 1: {
      int p[4], pos=0;
      for(int i=0;i<4;i++) p[i]=nextInt(pos);
      arm_.runtoPos(p[0],p[1],p[2],p[3]);
      arm_.waitForMotionComplete();
      break;
    }
    case 2: {
      idx = content.indexOf(',');
      arm_.setVel(content.substring(0,idx).toInt(),
                  content.substring(idx+1).toInt());
      break;
    }
    case 3: {
      int pos=0, x=nextInt(pos), y=nextInt(pos), z=nextInt(pos);
      arm_.runtoPosxy(x,y,z,0);
      arm_.waitForMotionComplete();
      break;
    }
    case 4: {
      idx = content.indexOf(',');
      int x=content.substring(0,idx).toInt();
      int y=content.substring(idx+1).toInt();
      arm_.runtoPosxy(x,y,arm_.getCurrentZ(),0);
      arm_.waitForMotionComplete();
      break;
    }
    case 5: {
      int pos=0, len=nextInt(pos);
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=arm_.getCurrentZ();
        path[i][3]=arm_.getJoint3Angle();
      }
      arm_.executePath(path,len);
      break;
    }
    case 6: {
      int pos=0, len=nextInt(pos);
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][3]=arm_.getJoint3Angle();
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=nextInt(pos);
      }
      arm_.executePath(path,len);
      break;
    }
    case 7: {
      int pos=0, len=nextInt(pos);
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=nextInt(pos);
        path[i][3]=nextInt(pos);
      }
      arm_.executePath(path,len);
      break;
    }
    case 8: {
      int pos=0, len=nextInt(pos);
      long th3 = arm_.getJoint3Angle();
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=nextInt(pos);
        path[i][3]=th3;
      }
      arm_.runThroughPath(path,len,20);
      break;
    }
    case 9: {
      int pos=0, len=nextInt(pos);
      int path[len][4];
      for(int i=0;i<len;i++){
        path[i][0]=nextInt(pos);
        path[i][1]=nextInt(pos);
        path[i][2]=nextInt(pos);
        path[i][3]=nextInt(pos);
      }
      arm_.runThroughPath(path,len,20);
      break;
    }
    case 10: {
      char key = content.charAt(0);
      int i = key - 'A';
      if (i>=0 && i<11) {
        if (charActions_[i]) {
          charActions_[i]();
        }
        else if (charSequences_[i].length()) {
          runSequence(charSequences_[i]);
          return;  // runSequence prints its own final "done"
        }
      }
      break;
    }
  }
  arm_.serialFlush();
  flushInput();
}

void ScaraController::runSequence(const String &seq) {
  int start = 0;
  while (start < seq.length()) {
    int semi = seq.indexOf(';', start);
    String piece = semi<0
      ? seq.substring(start)
      : seq.substring(start, semi);
    piece.trim();
    if (piece.length()) executeSingle(piece);
    if (semi<0) break;
    start = semi + 1;
  }
  Serial.println("done");
  arm_.serialFlush();
}

void ScaraController::flushInput() {
  while (Serial.available()) Serial.read();
}
