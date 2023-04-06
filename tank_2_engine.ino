#include <GyverMotor.h>
#include <SoftwareSerial.h>

#define PIN_MOTOR_OFF_ON 2

#define PIN_MOTOR_RA 7
#define PIN_MOTOR_RB 8
#define PIN_MOTOR_R_PWM 9

#define PIN_MOTOR_LA 3
#define PIN_MOTOR_LB 4
#define PIN_MOTOR_L_PWM 10

#define MOTOR_EMPTY_ZONE 20

#define MOTOR_MAX 255
#define JOYSTICK_MAX 255

#define BT_TX 12
#define BT_RX 11

#define END_PACKAGE ';'
#define Y_SYMBOL 'Y'
#define X_SYMBOL 'X'
#define PING_SYMBOL 'S'

#define TIMEOUT_PING 700

GMotor motorR(DRIVER3WIRE, PIN_MOTOR_RA, PIN_MOTOR_RB, PIN_MOTOR_R_PWM);
GMotor motorL(DRIVER3WIRE, PIN_MOTOR_LA, PIN_MOTOR_LB, PIN_MOTOR_L_PWM);

SoftwareSerial btSerial(BT_TX, BT_RX);

boolean doneParsing;
int dataX, dataY;
int speedEngineR, speedEngineL;

unsigned long lastTimeParsing;

void setup() {
  // Пины D9 и D10 - 31.4 кГц
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00000001;  // x1 phase correct

  Serial.begin(9600);
  btSerial.begin(9600);

  pinMode(PIN_MOTOR_OFF_ON, OUTPUT);
  digitalWrite(PIN_MOTOR_OFF_ON, HIGH);

  motorL.setMinDuty(MOTOR_EMPTY_ZONE);
  motorL.setMode(AUTO);

  motorR.setMinDuty(MOTOR_EMPTY_ZONE);
  motorR.setMode(AUTO);

  digitalWrite(13, LOW);
}

void parsing() {
  if (btSerial.available() > 0) {
    String buf = btSerial.readStringUntil(END_PACKAGE);
    Serial.println(buf);

    if (buf[0] == X_SYMBOL) {
      int indexStartY = buf.indexOf(Y_SYMBOL);

      dataX = buf.substring(1, indexStartY).toInt();
      dataY = buf.substring(indexStartY + 1).toInt();
    }

    if (buf[0] == X_SYMBOL || buf[0] == PING_SYMBOL) {
      lastTimeParsing = millis();
      doneParsing = true;
    }
  }
}

void loop() {
  parsing();
  computeSpeed();   
  setSpeed();
  interruptIfLostConnection();
}

void interruptIfLostConnection() {
  if (millis() - lastTimeParsing >= TIMEOUT_PING) {
    dataX = 0;
    dataY = 0;
    doneParsing = true;
  }
}

void computeSpeed() {
  if (doneParsing) {
    doneParsing = false;
    
    speedEngineR = dataY - dataX;
    speedEngineL = dataY + dataX;
  }
}

void setSpeed() {
  motorR.setSpeed(speedEngineR);
  motorL.setSpeed(speedEngineL);
}
