#include <Servo.h>
#include <SoftwareSerial.h>

// ----------- Bluetooth Module -----------
#define BT_RX A4
#define BT_TX A5
SoftwareSerial BT(BT_RX, BT_TX);

// ----------- PIN DEFINITIONS -----------
// Lane 1 LEDs and Servo
#define L1_R 2
#define L1_Y 3
#define L1_G 4
#define L1_SERVO A2

// Lane 2 LEDs and Servo
#define L2_R 8
#define L2_Y 9
#define L2_G 10
#define L2_SERVO A1

// Lane 3 LEDs and Servo
#define L3_R 5
#define L3_Y 6
#define L3_G 7
#define L3_SERVO A0

// Lane 4 LEDs and Servo
#define L4_R 11
#define L4_Y 12
#define L4_G 13
#define L4_SERVO A3

// ----------- TIMINGS (ms) -----------
#define PRE_GREEN 2000
#define GREEN_TIME 5000
#define POST_GREEN 2000
#define AMBULANCE_TIME 10000 // 10 sec override

// ----------- SERVOS -----------
Servo lane1Servo;
Servo lane2Servo;
Servo lane3Servo;
Servo lane4Servo;

// ----------- HELPER FUNCTION -----------
void setLane(int R, int Y, int G, bool red, bool yellow, bool green){
  digitalWrite(R, red ? HIGH : LOW);
  digitalWrite(Y, yellow ? HIGH : LOW);
  digitalWrite(G, green ? HIGH : LOW);
}

// ----------- LANE STATES -----------
enum LaneState {RED_STATE, PRE_GREEN_STATE, GREEN_STATE, POST_GREEN_STATE};

struct Lane {
  int R, Y, G;
  Servo* servo;
  LaneState state;
  unsigned long timer;
};

// ----------- INITIALIZE LANES -----------
Lane lanes[4];

// ----------- AMBULANCE MODE VARIABLES -----------
bool ambulanceMode = false;
int ambulanceLane = -1;
unsigned long ambulanceTimer = 0;

// ----------- CUSTOM SERVO MAPPING -----------
// Light 1 → Servo 4
// Light 2 → Servo 1
// Light 3 → Servo 2
// Light 4 → Servo 3
Servo* servoMap[4];

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  int leds[] = {L1_R, L1_Y, L1_G, L2_R, L2_Y, L2_G, L3_R, L3_Y, L3_G, L4_R, L4_Y, L4_G};
  for (int i = 0; i < 12; i++) pinMode(leds[i], OUTPUT);

  lane1Servo.attach(L1_SERVO);
  lane2Servo.attach(L2_SERVO);
  lane3Servo.attach(L3_SERVO);
  lane4Servo.attach(L4_SERVO);

  // Servo mapping
  servoMap[0] = &lane4Servo; // light 1 → servo 4
  servoMap[1] = &lane1Servo; // light 2 → servo 1
  servoMap[2] = &lane2Servo; // light 3 → servo 2
  servoMap[3] = &lane3Servo; // light 4 → servo 3

  // Initialize lane data
  lanes[0] = {L1_R, L1_Y, L1_G, &lane1Servo, PRE_GREEN_STATE, millis()};
  lanes[1] = {L2_R, L2_Y, L2_G, &lane2Servo, RED_STATE, 0};
  lanes[2] = {L3_R, L3_Y, L3_G, &lane3Servo, RED_STATE, 0};
  lanes[3] = {L4_R, L4_Y, L4_G, &lane4Servo, RED_STATE, 0};

  // Set all red + close all servos
  for (int i = 0; i < 4; i++) {
    setLane(lanes[i].R, lanes[i].Y, lanes[i].G, true, false, false);
    lanes[i].servo->write(0);
  }
}

void activateAmbulanceLane(int laneNum){
  ambulanceMode = true;
  ambulanceLane = laneNum;
  ambulanceTimer = millis();

  for(int i=0;i<4;i++){
    setLane(lanes[i].R, lanes[i].Y, lanes[i].G, true, false, false);
    lanes[i].servo->write(0);
  }

  // Open servo according to mapping
  setLane(lanes[laneNum].R, lanes[laneNum].Y, lanes[laneNum].G, false, false, true);
  servoMap[laneNum]->write(90);

  Serial.print("🚑 Ambulance lane activated: "); 
  Serial.println(laneNum+1);
}

void loop() {
  unsigned long current = millis();

  // ----- Bluetooth Check -----
  if(BT.available()){
    char c = BT.read();
    if(c >= '1' && c <= '4'){
      int laneNum = c - '1';
      activateAmbulanceLane(laneNum);
    }
  }

  // ----- Ambulance Mode -----
  if(ambulanceMode){
    if(current - ambulanceTimer >= AMBULANCE_TIME){
      ambulanceMode = false;
      Serial.println("✅ Ambulance mode over, resuming normal cycle");
      for(int i=0;i<4;i++){
        if(lanes[i].state != RED_STATE) lanes[i].timer = current;
      }
    } else return;
  }

  // ----- Normal Traffic Flow -----
  for(int i=0;i<4;i++){
    Lane &l = lanes[i];
    int nextLane = (i+1)%4;

    switch(l.state){
      case RED_STATE:
        setLane(l.R, l.Y, l.G, true, false, false);
        break;

      case PRE_GREEN_STATE:
        setLane(l.R, l.Y, l.G, true, true, false);
        if(current - l.timer >= PRE_GREEN){
          l.state = GREEN_STATE;
          l.timer = current;
        }
        break;

      case GREEN_STATE:
        setLane(l.R, l.Y, l.G, false, false, true);
        // open mapped servo for this light
        servoMap[i]->write(90);
        if(current - l.timer >= GREEN_TIME){
          l.state = POST_GREEN_STATE;
          l.timer = current;
          if(lanes[nextLane].state == RED_STATE){
            lanes[nextLane].state = PRE_GREEN_STATE;
            lanes[nextLane].timer = current;
          }
        }
        break;

      case POST_GREEN_STATE:
        setLane(l.R, l.Y, l.G, false, true, false);
        if(current - l.timer >= POST_GREEN){
          l.state = RED_STATE;
          l.timer = 0;
          // close mapped servo for this light
          servoMap[i]->write(0);
          if(lanes[nextLane].state == PRE_GREEN_STATE){
            lanes[nextLane].state = GREEN_STATE;
            lanes[nextLane].timer = current;
          }
        }
        break;
    }
  }
}
