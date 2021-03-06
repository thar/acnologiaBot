#include <SharpDistSensor.h>
#include <AStar32U4.h>
#include <QTRSensors.h>

#define LEFT_DISTANCE_A_PIN A0
#define LEFT_DISTANCE_D_PIN 4
#define RIGHT_DISTANCE_A_PIN A1
#define RIGHT_DISTANCE_D_PIN 5

#define LEFT_FLOOR_SENSOR_PIN 8
#define RIGHT_FLOOR_SENSOR_PIN 7

#define HALF_TURN_TIME 1000
#define FULL_BACKWARD_TIME 1000
#define SEARCH_TIME 1000

#define FLOOR_SENSOR_THRESHOLD 2000

#define MIN_DISTANCE_FOR_OBSTACLE 100
#define MAX_DISTANCE_FOR_OBSTACLE 700

#define KP 10
#define KD 100

#define SPEED_REDUCTION 0

AStar32U4ButtonA buttonA;
AStar32U4ButtonB buttonB;
AStar32U4ButtonC buttonC;
AStar32U4Motors motors;

// Window size of the median filter (odd number, 1 = no filtering)
const byte mediumFilterWindowSize = 5;

// Create an object instance of the SharpDistSensor class
SharpDistSensor leftSensor(LEFT_DISTANCE_A_PIN, mediumFilterWindowSize);
SharpDistSensor rightSensor(RIGHT_DISTANCE_A_PIN, mediumFilterWindowSize);

QTRSensorsRC qtrrc((unsigned char[]) {LEFT_FLOOR_SENSOR_PIN, RIGHT_FLOOR_SENSOR_PIN}, 2);
unsigned int sensor_values[2];


void fixPwmMaxValues(int &linealPwm, int &angularPwm) {
    int maxPwm = abs(angularPwm) + abs(linealPwm);
    static int minPwmLineal = 200;
    if (maxPwm > 400) {
        int linealMax = abs(linealPwm);
        int linealMargin = linealMax - minPwmLineal;
        int angularMax = abs(angularPwm);
        int extra = 400 - maxPwm;
        if (linealMargin > 0) {
            int linealReduction = min(linealMargin, extra);
            linealMax -= linealReduction;
            extra -= linealReduction;
        }
        if (extra > 0) {
            angularMax -= extra;
        }
        angularPwm = angularPwm > angularMax ? angularMax : angularPwm;
        angularPwm = angularPwm < -angularMax ? -angularMax : angularPwm;
        linealPwm = linealPwm > linealMax ? linealMax : linealPwm;
        linealPwm = linealPwm < -linealMax ? -linealMax : linealPwm;
    }
}


// positives turns move to left
void setSpeeds(int forward, int turn) {
    fixPwmMaxValues(forward, turn);
    if (0 != SPEED_REDUCTION) {
        forward *= 100 - SPEED_REDUCTION;
        turn *= 100 - SPEED_REDUCTION;
        forward /= 100;
        turn /= 100;
    }
    motors.setM1Speed(forward - turn);
    motors.setM2Speed(forward + turn);
}


bool isFloorSensorDetecting(unsigned int sensorValue) {
  //return false;
    if (sensorValue > FLOOR_SENSOR_THRESHOLD) {
        return false;
    } else {
        return true;
    }
}


bool isObstacleSensorDetecting(unsigned int distance) {
    if (MIN_DISTANCE_FOR_OBSTACLE < distance && MAX_DISTANCE_FOR_OBSTACLE > distance) {
        return true;
    } else {
        return false;
    }
}


class MinisumoSensors {
public:
    void updateState() {
        qtrrc.read(sensor_values);
        leftDistance = leftSensor.getDist();
        rightDistance = rightSensor.getDist();
        leftFloorSensorDetected = isFloorSensorDetecting(sensor_values[0]);
        rightFloorSensorDetected = isFloorSensorDetecting(sensor_values[1]);
        leftObstaclePresent = isObstacleSensorDetecting(leftDistance);
        rightObstaclePresent = isObstacleSensorDetecting(rightDistance);
        if (leftObstaclePresent && !rightObstaclePresent) {
            lastSeen = -1;
        } else if (rightObstaclePresent && !leftObstaclePresent) {
            lastSeen = 1;
        }
        /*
        Serial.print(leftDistance);
        Serial.print(',');
        Serial.print(rightDistance);
        Serial.print(',');
        Serial.print(sensor_values[0]);
        Serial.print(',');
        Serial.println(sensor_values[1]);
        */
    }
    bool isSomeObstacleDetected() {
        return leftObstaclePresent || rightObstaclePresent;
    }
    bool isSomeFloorSensorDetected() {
        return leftFloorSensorDetected || rightFloorSensorDetected;
    }

    unsigned int leftDistance = 0;
    unsigned int rightDistance = 0;
    bool leftFloorSensorDetected = false;
    bool rightFloorSensorDetected = false;
    bool leftObstaclePresent = false;
    bool rightObstaclePresent = false;
    int lastSeen = 0;
    unsigned int sensors[2];

protected:
private:

};


class MinisumoAction;
MinisumoAction* getActionFromSensors(MinisumoSensors& sensors);

class MinisumoActionContext {
public:
    void setMinisumoAction(MinisumoAction *minisumoAction);
    void step();

private:
    MinisumoAction *minisumoAction_;
    MinisumoSensors sensors;

};


class MinisumoAction {
public:
    void step(MinisumoSensors& sensors) {
        if (isAnyButtonPressed()) {
            while(1) {
                setSpeeds(0, 0);
            }
        }
        doStep(sensors);
    }
    virtual void doStep(MinisumoSensors& sensors) = 0;
    virtual ~MinisumoAction() {}
    bool isAnyButtonPressed() {
        if (buttonA.getSingleDebouncedRelease() || buttonB.getSingleDebouncedRelease() || buttonC.getSingleDebouncedRelease()) {
            return true;
        } else {
            return false;
        }
    }
    void setContext(MinisumoActionContext * context) {
        context_ = context;
    }
protected:
    MinisumoActionContext * context_;
    
private:
};


class BothFloorSensorsDetected : public MinisumoAction {
public:
    BothFloorSensorsDetected() : startTime_(millis()) {
        setSpeeds(-400, 0);
    }
    void doStep(MinisumoSensors& sensors);
protected:
private:
    long startTime_;
};


class OneFloorSensorsDetected : public MinisumoAction {
public:
    OneFloorSensorsDetected(int direction) : stopTime_(millis() + HALF_TURN_TIME) {
        setSpeeds(-300, direction * 100);
    }
    void doStep(MinisumoSensors& sensors);

protected:
private:
    long stopTime_;
};


class TimedTurn : public MinisumoAction {
public:
    TimedTurn(int direction, unsigned int time) : stopTime_(millis() + time) {
        setSpeeds(0, direction * 400);
    }
    void doStep(MinisumoSensors& sensors);

protected:
private:
    long stopTime_;
};


class ForwardTilBorder : public MinisumoAction {
public:
    ForwardTilBorder() {
        setSpeeds(400, 0);
    }
    void doStep(MinisumoSensors& sensors);

protected:
private:
};


class OnlyLeftPresetAction : public MinisumoAction {
public:
    OnlyLeftPresetAction() {
        setSpeeds(200, 100);
    }
    void doStep(MinisumoSensors& sensors);

protected:
private:
};


class OnlyRightPresetAction : public MinisumoAction {
public:
    OnlyRightPresetAction() {
        setSpeeds(200, -100);
    }
    void doStep(MinisumoSensors& sensors);

protected:
private:
};


class AttackPidAction : public MinisumoAction {
public:
    void doStep(MinisumoSensors& sensors);

protected:
private:
    long oldDifference = 0;
};


void MinisumoActionContext::step() {
    sensors.updateState();
    if (nullptr == minisumoAction_)
        setMinisumoAction(getActionFromSensors(sensors));
    minisumoAction_->step(sensors);
}


void MinisumoActionContext::setMinisumoAction(MinisumoAction *minisumoAction) {
    if (nullptr != minisumoAction_) {
        delete minisumoAction_;
    }
    minisumoAction_ = minisumoAction;
    minisumoAction_->setContext(this);
}


MinisumoAction* getActionFromSensors(MinisumoSensors& sensors) {
    if (sensors.leftFloorSensorDetected && sensors.rightFloorSensorDetected) { // both floor sensors detects
        Serial.println("BothFloorSensorsDetected");
        return new BothFloorSensorsDetected();
    } else if (sensors.leftFloorSensorDetected) { // left floor sensor detects
        Serial.println("OneFloorSensorsDetected(-1) L");
        return new OneFloorSensorsDetected(-1);
    } else if (sensors.rightFloorSensorDetected) { // right floor sensor detects
        Serial.println("OneFloorSensorsDetected(1) R");
        return new OneFloorSensorsDetected(1);
    } else if (!sensors.isSomeObstacleDetected()) {
        switch(sensors.lastSeen) {
        case -1:
            Serial.println("TimedTurn L");
            return new TimedTurn(1, SEARCH_TIME);
            break;
        case 1:
            Serial.println("TimedTurn R");
            return new TimedTurn(-1, SEARCH_TIME);
            break;
        default:
            Serial.println("ForwardTilBorder");
            return new ForwardTilBorder();
            break;
        }
    } else {
        if (!sensors.leftObstaclePresent) {
            Serial.println("OnlyRightPresetAction");
            return new OnlyRightPresetAction();
        } else if (!sensors.rightObstaclePresent) {
            Serial.println("OnlyLeftPresetAction");
            return new OnlyLeftPresetAction();
        } else {
            Serial.println("AttackPidAction");
            return new AttackPidAction();
        }
    }
}


void BothFloorSensorsDetected::doStep(MinisumoSensors& sensors) {
    long elapsedTime = millis() - startTime_;
    if (elapsedTime > FULL_BACKWARD_TIME) {
        Serial.println("From BothFloorSensorsDetected to TimedTurn");
        context_->setMinisumoAction(new TimedTurn(sensors.lastSeen < 0 ? 1 : -1, HALF_TURN_TIME));
    } else if (!sensors.isSomeFloorSensorDetected() && sensors.isSomeObstacleDetected()) {
      Serial.print("From BothFloorSensorsDetected to ");
        context_->setMinisumoAction(getActionFromSensors(sensors));
    }
}


void OneFloorSensorsDetected::doStep(MinisumoSensors& sensors) {
    long actualTime = millis();
    if (actualTime > stopTime_ || !sensors.isSomeFloorSensorDetected() && sensors.isSomeObstacleDetected()) {
      Serial.print("From OneFloorSensorsDetected to ");
        context_->setMinisumoAction(getActionFromSensors(sensors));
    }
}


void TimedTurn::doStep(MinisumoSensors& sensors) {
    long actualTime = millis();
    if (actualTime > stopTime_) {
        Serial.println("From TimedTurn to ForwardTilBorder");
        context_->setMinisumoAction(new ForwardTilBorder());
    } else if (!sensors.isSomeFloorSensorDetected() && sensors.isSomeObstacleDetected()) {
      Serial.print("From TimedTurn to ");
        context_->setMinisumoAction(getActionFromSensors(sensors));
    }
}


void ForwardTilBorder::doStep(MinisumoSensors& sensors) {
    if (sensors.isSomeFloorSensorDetected() || sensors.isSomeObstacleDetected()) {
      Serial.print("From ForwardTilBorder to ");
        context_->setMinisumoAction(getActionFromSensors(sensors));
    }
}


void OnlyLeftPresetAction::doStep(MinisumoSensors& sensors) {
    if (sensors.isSomeFloorSensorDetected() || !sensors.leftObstaclePresent || sensors.rightObstaclePresent) {
      Serial.print("From OnlyLeftPresetAction to ");
        context_->setMinisumoAction(getActionFromSensors(sensors));
    }
}


void OnlyRightPresetAction::doStep(MinisumoSensors& sensors) {
    if (sensors.isSomeFloorSensorDetected() || !sensors.rightObstaclePresent || sensors.leftObstaclePresent) {
      Serial.print("From OnlyRightPresetAction to ");
        context_->setMinisumoAction(getActionFromSensors(sensors));
    }
}


void AttackPidAction::doStep(MinisumoSensors& sensors) {
    if (!sensors.leftObstaclePresent || !sensors.rightObstaclePresent) {
      Serial.print("From AttackPidAction to ");
        context_->setMinisumoAction(getActionFromSensors(sensors));
    } else {
        long difference = static_cast<long>(sensors.rightDistance) - static_cast<long>(sensors.leftDistance);
        long angularSpeed = KP * difference + KD * (difference - oldDifference);
        oldDifference = difference;
        setSpeeds(400, angularSpeed/100);
    }
}


MinisumoActionContext minisumocontext;


void setup() {
  //motors.flipM1(true);
  //motors.flipM2(true);
  Serial.begin(115200);
  pinMode(LEFT_DISTANCE_D_PIN, OUTPUT);
  pinMode(RIGHT_DISTANCE_D_PIN, OUTPUT);
  digitalWrite(LEFT_DISTANCE_D_PIN, HIGH);
  digitalWrite(RIGHT_DISTANCE_D_PIN, HIGH);
  setSpeeds(0, 0);
  ledRed(0);
  ledYellow(0);
  ledGreen(1);
  buttonA.waitForButton();
  for (int i=0; i<4; ++i) {
    ledRed(1);
    delay(500);
    ledRed(0);
    delay(500);
  }
  ledYellow(1);
  delay(500);
  ledYellow(0);
  delay(500);
  ledGreen(1);
}

void loop() {
    minisumocontext.step();
}

