/*
  Wireless Servo Control, with ESP as Access Point

  Usage:
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1.
    A webpage with four buttons should appear. Click them to move the robot.

  Installation:
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets

    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware:
    NodeMCU Amica DevKit Board (ESP8266 chip)
    Motorshield for NodeMCU
    2 continuous rotation servos plugged into motorshield pins D1, D2
    Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>
#include <Servo.h>

#include "debug.h"
#include "file.h"
#include "server.h"


const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
const double pi = 3.14159;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;
int sensor_left_ct, last_left;
int sensor_right_ct, last_right;
double state[6]; // vx, vy, w, x, y, theta
double change_state[3]; // dx,dy,dtheta
int current_millis_l;
int current_millis_r;
char stat = 'S';
double A[6][6];
double A_T[6][6];
double B[6][2];
double C[3][6];
double C_T[6][3];
double U[2];  // PWM define later
double P[6][6];
double R[3][3];
int period;
double G[6][3];
double Z[3]; // Z[2] needed to define later, For gyro??

// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid =
  "...";
char* sta_password =
  "...";

char* mDNS_name = "paperbot";

String html;
String css;

void setup() {
  setupPins();

  sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
    Serial.flush();
    LED_ON;
    delay(500);
    LED_OFF;
    delay(500);
  }
  LED_ON;
  //setupSTA(sta_ssid, sta_password);
  setupAP(ap_ssid, ap_password);
  LED_OFF;

  pinMode(D3, INPUT);
  pinMode(D4, INPUT);
  sensor_left_ct = 0;
  sensor_right_ct = 0;
  last_left = 0;
  last_right = 0;
  state[0] = state[1] = state[2] = state[3] = state[4] = state[5] = 0;

  setupFile();
  html = loadFile("/controls.html");
  css = loadFile("/style.css");
  registerPage("/", "text/html", html);
  registerPage("/style.css", "text/css", css);

  setupHTTP();
  setupWS(webSocketEvent);
  //setupMDNS(mDNS_name);

  stop();
}

void loop() {
  period = millis();
  wsLoop();
  httpLoop();
  rotationLoop();
  period -= millis();
  period = ~period + 1;

  matrixAFill(period / 1000); /// NEED T
  matrixBFill(state[5]);
  matrixCFill(state[0], state[1]);
  matrixPFill(0.5);
  matrixRFill();
  matrixMul1();
  matrixMul2();
  matrixMul3();
  matrixMul4();
  matrixMul5();
}



void rotationLoop() {
  if (last_left != digitalRead(D3)) { // left moving
    sensor_left_ct++;
    if (sensor_left_ct % 5 == 0 && sensor_left_ct % 10 != 0) {
      current_millis_l = millis();
    }
    else if (sensor_left_ct % 5 == 0 && sensor_left_ct % 10 == 0) {
      Z[1] = (6 * pi) / 4 / (millis() - current_millis_l) * 1000;
    }
    last_left = digitalRead(D3);
  }
  if (last_right != digitalRead(D4)) {
    sensor_right_ct++;
    if (sensor_right_ct % 5 == 0 && sensor_right_ct % 10 != 0) {
      current_millis_r = millis();
    }
    else if (sensor_right_ct % 5 == 0 && sensor_right_ct % 10 == 0) {
      Z[0] = (6 * pi) / 4 / (millis() - current_millis_l) * 1000;
    }
    last_right = digitalRead(D4);
  }

  int16_t gx;
  int16_t gy;
  int16_t gz;
  getGyroData(&gx, &gy, &gz);
  Z[2] = (int) gz;
  
  switch (stat) {// vx, vy, w, x, y, theta
    case 'S':
      state[0] = state[1] = state[2] = 0;
      sensor_left_ct = sensor_right_ct = 0;
      state[3] += change_state[0];
      state[4] += change_state[1];
      state[5] += change_state[2];
      change_state[0] = change_state[1] = change_state[2] = 0;
      break;

    case 'F':
      state[0] = Z[1] * cos(state[5]);
      state[1] = Z[1] * sin(state[5]);
      change_state[0] = sensor_right_ct * (6 * pi / 20) * cos(state[5]);
      change_state[1] = sensor_right_ct * (6 * pi / 20) * sin(state[5]);
      break;

    case 'B':
      state[0] = Z[1] * cos(state[5] + pi);
      state[1] = Z[1] * sin(state[5] + pi);
      change_state[0] = sensor_right_ct * (6 * pi / 20) * cos(state[5] + pi);
      change_state[1] = sensor_right_ct * (6 * pi / 20) * sin(state[5] + pi);
      break;

    case 'L':
      state[2] = 2 * Z[0] / 8.5; // d=8.5
      change_state[2] = sensor_right_ct * (6 * pi / 20) / (8.5 / 2);
      break;

    case 'R':
      state[2] = 2 * Z[1] / 8.5; // d=8.5
      change_state[2] = sensor_left_ct * (6 * pi / 20) / (-8.5 / 2);
      break;

  }
}


void matrixAFill(double t) {
  int i;
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      A[i][j] = 0;
    }
  }
  A[3][0] = A[4][1] = A[5][2] = t;
  A[3][3] = A[4][4] = A[5][5] = 1;
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      A_T[j][i] = A[i][j];
    }
  }
}

void matrixBFill(double theta) {
  B[0][0] = B[0][1] = sin(theta) / 2;
  B[1][0] = B[1][1] = cos(theta) / 2;
  B[2][0] = 1 / 8.5;
  B[2][1] = -1 / 8.5;
  B[3][0] = B[3][1] = B[4][0] = B[4][1] = B[5][0] = B[5][1];
}

void matrixCFill(double vx, double vy) {
  int i;
  switch (stat) {
    case 'B':
    case 'F':
      C[0][0] = C[1][0] = vx / sqrt(vx * vx + vy * vy);
      C[0][1] = C[1][1] = vy / sqrt(vx * vx + vy * vy);
      C[2][0] = C[2][1] = 0;
      for (i = 0; i < 3; i++) {
        int j;
        for (j = 2; j < 6; j++) {
          C[i][j] = 0;
        }
      }
      break;
    case 'L':
    case 'R':
      for (i = 0; i < 3; i++) {
        int j;
        for (j = 0; j < 6; j++) {
          C[i][j] = 0;
          if (i == 0 && j == 2) {
            if (stat == 'L')
              C[0][2] = 8.5 / 2;
            if (stat == 'R')
              C[0][2] = -8.5 / 2;
          }
        }
      }
      C[1][2] = 8.5 / 2;
      C[2][2] = 1;
      break;
    case 'S':
      for (i = 0; i < 3; i++) {
        int j;
        for (j = 0; j < 6; j++) {
          C[i][j] = 0;
        }
      }
      break;
  }

  for (i = 0; i < 3; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      C_T[j][i] = C[i][j];
    }
  }
}

void matrixPFill(double P_factor) {
  int i;
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      P[i][j] = 0;
      if (i == j) {
        P[i][j] = P_factor;
      }
    }
  }
}

void matrixRFill() {
  int i;
  for (i = 0; i < 3; i++) {
    int j;
    for (j = 0; j < 3; j++) {
      R[i][j] = 0;
      if (i == j)
        R[i][j] = 1;
    }
  }
}

void matrixMul1() {
  double ans[6];
  for (int i = 0; i < 6; i++) {
    ans[i] = 0;
    for (int j = 0; j < 6; j++) {
      ans[i] += A[i][j] * state[j];
    }
  }
  double temp[6];
  for (int i = 0; i < 6; i++) {
    temp[i] = 0;
    for (int j = 2; j < 2; j++) {
      temp[i] += B[i][j] * U[j];
    }
  }
  for (int i = 0; i < 6; i++) {
    ans[i] += temp[i];
    state[i] = ans[i];
  }
}

void matrixMul2() {
  double temp[6][6];
  int i;
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      int k;
      temp[i][j] = 0;
      for (k = 0; k < 6; k++) {
        temp[i][j] += A[i][k] * P[k][j];
      }
    }
  }


  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      int k;
       P[i][j] = 0;
      for (k = 0; k < 6; k++) {
        P[i][j] += temp[i][k] * A_T[k][j];
      }
    }
  }
}

void matrixMul3() { // P 6x6 C_T 6x3
  double temp[6][3];
  int i;
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 3; j++) {
      temp[i][j] = 0;
      int k;
      for (k = 0; k < 6; k++) {
        temp[i][j] += P[i][k] * C_T[k][j];
      }
    }
  }
  double CP[3][6];
  Matrix_Multiply(&C[0][0], &P[0][0],3,6,6,&CP[0][0]);
  double CPC[3][3];
  Matrix_Multiply(&CP[0][0], &C_T[0][0], 3,6,3,&CPC[0][0]);
  double CPCR[3][3];
  Matrix_Add(&CP[0][0], &R[0][0],3,3,&CPCR[0][0]);
  Matrix_Inverse(&CPCR[0][0],3);
  Matrix_Multiply(&temp[0][0],&CPCR[0][0],6,3,3,&G[0][0]);
}
  

void matrixMul4() { // x=x+G(z-Cx)
  //Cx 3x6 * 6x1
  double Cx[3];
  int i;
  for (i = 0; i < 3; i++) {
    Cx[i] = 0;
    int k;
    for (k = 0; k < 6; k++) {
      Cx[i] += C[i][k] * state[k];
    }
  }
  // z-Cx
  for (i = 0; i < 3; i++) {
    Cx[i] = Z[i] - Cx[i];
  }
  //G(z-Cx) 6x3 * 3x1
  double temp[6];
  for (i = 0; i < 6; i++) {
    temp[i] = 0;
    int k;
    for (k = 0; k < 3; k++) {
      temp[i] += G[i][k] * Cx[k];
    }
  }
  // x+temp
  for (i = 0; i < 6; i++) {
    state[i] += temp[i];
  }
}

void matrixMul5() { // P=(I-GC)P
  double temp[6][6];
  int i;
  //GC 6x3 * 3*6
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      temp[i][j] = 0;
      int k;
      for (k = 0; k < 3; k++) {
        temp[i][j] += G[i][k] * C[k][j];
      }
    }
  }
  // define I
  int I[6][6];
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      I[i][j] = 0;
      if (i == j)
        I[i][j] = 1;
    }
  }
  //I-GC
  for(i=0;i<6;i++){
    int j;
    for(j=0;j<6;j++){
      temp[i][j] = I[i][j] - temp[i][j];
    }
  }
  // P=(I-GC)P reuse I as new P here
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      int k;
       I[i][j] = 0;
      for (k = 0; k < 6; k++) {
        I[i][j] += temp[i][k] * P[k][j];
      }
    }
  }
  // put value back to P
  for (i = 0; i < 6; i++) {
    int j;
    for (j = 0; j < 6; j++) {
      P[i][j] = I[i][j];
    }
  }
}


//
// Movement Functions //
//

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  stat = 'S';
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  stat = 'F';
  DEBUG("forward");
  drive(0, 180);
}

void backward() {
  stat = 'B';
  DEBUG("backward");
  drive(180, 0);
}

void left() {
  stat = 'L';
  DEBUG("left");
  drive(180, 180);
}

void right() {
  stat = 'R';
  DEBUG("right");
  drive(0, 0);
}



//
// Setup //
//

void setupPins() {
  // setup Serial, LEDs and Motors
  Serial.begin(115200);
  DEBUG("Started serial.");

  pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
  LED_OFF;                     //Turn off LED
  DEBUG("Setup LED pin.");

  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);
  DEBUG("Setup motor pins");

}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

  char a[20];
  sprintf(a, "x : %d", (int)state[3]);
  wsSend(id, a);
  sprintf(a, "y : %d", (int)state[4]);
  wsSend(id, a);
  sprintf(a, "Angle : %d", (int)state[5]);
  wsSend(id, a);
  
  //Test Code!!


  switch (type) {
    case WStype_DISCONNECTED:
      DEBUG("Web socket disconnected, id = ", id);
      break;
    case WStype_CONNECTED:
      {
        // IPAddress ip = webSocket.remoteIP(id);
        // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
        DEBUG("Web socket connected, id = ", id);

        // send message to client
        wsSend(id, "Connected to ");
        wsSend(id, ap_ssid);
        break;
      }
    case WStype_BIN:
      DEBUG("On connection #", id)
      DEBUG("  got binary of length ", length);
      for (int i = 0; i < length; i++)
        DEBUG("    char : ", payload[i]);

      if (payload[0] == '~')
        drive(180 - payload[1], payload[2]);

    case WStype_TEXT:
      DEBUG("On connection #", id)
      DEBUG("  got text: ", (char *)payload);

      if (payload[0] == '#') {
        if (payload[1] == 'C') {
          LED_ON;
          wsSend(id, "Hello world!");
        }
        else if (payload[1] == 'F')
          forward();
        else if (payload[1] == 'B')
          backward();
        else if (payload[1] == 'L')
          left();
        else if (payload[1] == 'R')
          right();
        else if (payload[1] == 'U') {
          if (payload[2] == 'L')
            servo_left_ctr -= 1;
          else if (payload[2] == 'R')
            servo_right_ctr += 1;
          char tx[20] = "Zero @ (xxx, xxx)";
          sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
          wsSend(id, tx);
        }
        else if (payload[1] == 'D') {
          if (payload[2] == 'L')
            servo_left_ctr += 1;
          else if (payload[2] == 'R')
            servo_right_ctr -= 1;
          char tx[20] = "Zero @ (xxx, xxx)";
          sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
          wsSend(id, tx);
        }
        else
          stop();
      }

      break;
  }
}
