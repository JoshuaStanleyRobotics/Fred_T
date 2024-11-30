 /*
   Description: This program is used to control a remote control hexagonal folding robot with differential drive

        Wiring: The required components are 6x MG90S servos, a NRF24L01 radio module, 2x DC motors, and an H-bridge motor driver.
        The servos are connected with brown wire to GND, red wire to Vin, and orange wire to D3, D4, D5, D6, D7, and D8
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D10, MOSI to D11, CE to D9, SCK to D13, MISO to D12
        The H-Bridge Driver is connected Vcc1 to 5V, Vcc2 to Vin, GND to GND, 1A to A0, 2A, to A1, 3A to A2, 4A to A3, 1Y and 2Y to A Motor, 3Y and 4Y to B Motor, 1, 2EN and 3,4EN to 5V
*/


//Libraries
#include "Servo.h"
#include "SPI.h"
#include "RF24.h"
#include "Ramp.h"

RF24 radio(9, 10);
Servo servo[2][3];
ramp angle[2][3];

//Variables
byte mode = 0;
byte cnt = 0;
int yDir = 0;
int left = 0;
int right = 0;
int drive = 0;
int duration = 0;
byte data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte but[][4] = {{2, 3, 6, 7}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
unsigned long millisPrev = 0;

//Constants
const int leftA = A0;
const int leftB = A1;
const int rightA = A3;
const int rightB = A2;

const byte pinout[2][3] = {{4, 6, 8}, {3, 5, 7}};                                                   //Array defining servo connections
const byte chan[6] = "00007";                                                                       //Radio channel used
const byte strt[2][3] = {{14, 7, 4}, {180, 169, 171}};                                              //Calibration array defining the written positions of each servo when the joint is straight
const byte perp[2][3] = {{109, 110, 100}, {82, 65, 70}};                                            //Calibration array defining the written positions of each servo when the joint is at a right angle
const byte flat[3] = {20, 180, 160};                                                                //Array with written position of each servo for the robot to go to its flat state
const byte tall[3] = {180, 90, 90};                                                                 //Array with written position of each servo for teh robot to go to its tall state

const byte seq[4][6][3] = {   /*[mode][cnt][servo]*/                                                //Array containing the servo angles to cycle through for each rolling gait
  {{150, 150, 60}, {60, 150, 150}, {150, 60, 150}, {150, 150, 60}, {60, 150, 150}, {150, 60, 150}},
  {{163, 163, 94}, {60, 120, 120}, {163, 94, 163}, {120, 120, 60}, {94, 163, 163}, {120, 60, 120}},
  {{180, 90, 90}, {180, 135, 45}, {90, 180, 90}, {45, 180, 135}, {90, 90, 180}, {135, 45, 180}},
  {{180, 90, 90}, {150, 150, 60}, {90, 180, 90}, {60, 150, 150}, {90, 90, 180}, {150, 60, 150}}
};

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Communication Initialized");

  radio.begin();                                                                                    //Begin radio communication
  radio.openReadingPipe(1, chan);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Radio Communication Initialized");

  pinMode(leftA, OUTPUT);                                                                           //Initialize motors
  pinMode(leftB, OUTPUT);
  pinMode(rightA, OUTPUT);
  pinMode(rightB, OUTPUT);
  Serial.println("Motors Attached");

  for (int i = 0; i < 2; i++) {                                                                     //Initialize servos
    for (int j = 0; j < 3; j++) {
      servo[i][j].attach(pinout[i][j]);
      angle[i][j].go(perp[i][j] - (perp[i][j] - strt[i][j]) / 3);
      servo[i][j].write(angle[i][j].update());
    }
  }
  Serial.println("Servos Attached");

  delay(250);
  Serial.println("Running");
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    debounce();

    if (data[1] == 0) yDir = 0;                                                                     //Forward and reverse controlled by front to back motion of left joystick
    else if (data[1] > 127) yDir = 1;
    else yDir = -1;

    if (data[4] == 0 && data[5] == 0) drive = 0;                                                    //Tread drive controlled by right joystick
    else {
      drive = 1;
      if (data[5] == 0) {
        if (data[4] > 127) {
          left = 1;
          right = 0;
        }
        else {
          left = 0;
          right = 1;
        }
      }
      else if (data[5] > 127) {
        left = 1;
        right = 1;
      }
      else {
        left = 0;
        right = 0;
      }
    }
    duration = map(data[9], 0, 255, 400, 100);                                                      //Driving speed controlled by right potentiometer
  }

  if (drive) {                                                                                      //Drive tread motors corresponding to commanded travel direction
    digitalWrite(leftA, left);
    digitalWrite(leftB, !left);
    digitalWrite(rightA, right);
    digitalWrite(rightB, !right);
  }
  else {
    digitalWrite(leftA, 0);
    digitalWrite(leftB, 0);
    digitalWrite(rightA, 0);
    digitalWrite(rightB, 0);
  }

  if (but[2][2]) {                                                                                  //Goes to flat state if right joystick is pressed
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++) {
        angle[i][j].go(map(flat[j], 90, 180, perp[i][j], strt[i][j]), duration);
      }
    }
  }
  else if (but[2][0]){                                                                              //Goes to tall state if left joystick is pressed
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++) {
        angle[i][j].go(map(tall[j], 90, 180, perp[i][j], strt[i][j]), 1000);
      }
    }
  }
  else if (but[2][3]) {                                                                             //Increment rolling gait mode if right trigger is pressed
    if (mode == 3) mode = 0;
    else mode++;
    cnt = 0;
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++) {
        angle[i][j].go(map(seq[mode][cnt][j], 90, 180, perp[i][j], strt[i][j]), duration);
      }
    }
  }
  else if (but[2][1]) {                                                                             //Decrement rolling gait [mode] if left trigger is pressed
    if (mode == 0) mode = 3;
    else mode--;
    cnt = 0;
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++) {
        angle[0][i].go(map(seq[mode][cnt][j], 90, 180, perp[i][j], strt[i][j]), duration);
      }
    }
  }
  

  if ((millis() - millisPrev) > angle[0][0].getDuration()) {                                        //If the previous shape change is complete:
    if (yDir != 0) {                                                                                  //Increment or decrement the shape [cnt] (depending on forward or reverse commanded)
      if (yDir > 0) {
        if (cnt == 5) cnt = 0;
        else cnt++;
      }
      else {
        if (cnt == 0) cnt = 5;
        else cnt--;
      }
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
          angle[i][j].go(map(seq[mode][cnt][j], 90, 180, perp[i][j], strt[i][j]), duration);          //Send ramp variable to angles of next shape in sequence
        }
      }
      millisPrev = millis();
    }
  }
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      if (angle[i][j].update() < 0) {
        servo[i][j].write(0);
      }
      else servo[i][j].write(angle[i][j].update());                                                 //Set servo angles to updated ramp angle value
    }
  }
}

void debounce() {                                                                                   //Causes momentary button inputs from controller to trigger on once when pressed and not for the duration of being pressed
  for (int i = 0; i < 4; i++) {
    if (data[but[0][i]]) {
      if (!but[1][i]) {
        but[2][i] = 1;
      }
      else but[2][i] = 0;
      but[1][i] = 1;
    }
    else {
      but[1][i] = 0;
      but[2][i] = 0;
    }
  }
}
