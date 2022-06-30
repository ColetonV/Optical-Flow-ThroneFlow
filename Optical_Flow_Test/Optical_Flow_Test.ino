#define HWSERIAL Serial4
//https://www.pjrc.com/teensy/td_uart.html
//TX pin 17
//RX pin 16
#include "Optical_Flow.h"
#include "EMAFilter.h"

Optical_Flow Flow;

unsigned long now = 0;
unsigned long flowTime = 0;
unsigned long printTime = 0;
int16_t x_motion = 0;
int16_t y_motion = 0;
int surface_quality = 0;

float initialTime;
float dtInitial;
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
bool newPulse = false;
const byte interruptPin = 22;
float Z_distance;
float dt;

EMAFilter x_motion_filter(0.01);
EMAFilter y_motion_filter(0.08);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), Pulse, CHANGE);
  while (!Serial) {
    delay(100);
  }
  HWSERIAL.begin(19200);
  while (!HWSERIAL) {
    delay(100);
  }
  flowTime = micros();
  initialTime = micros();
  dtInitial = micros();
}

void Pulse() {
  if (digitalRead(interruptPin) == HIGH) {
    // start measuring
    pulseInTimeBegin = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}

void loop() {
  Flow.read_buffer();

  now = micros();
  if (now - flowTime >= 15152) {
    Flow.update_flow();
    flowTime = now;
    x_motion = Flow.x_motion;
    y_motion = Flow.y_motion;
    float x_motion_filtered = x_motion_filter.filter(x_motion);
    float y_motion_filtered = y_motion_filter.filter(y_motion);
    //y_motion_filtered = y_motion_filter.filter(y_motion_filtered);
    surface_quality = Flow.surface_quality;
    float scalar = 1;
    float deg_rad = (42/2)*0.0174533;
    float y_motion_scaled = ((y_motion_filtered*Z_distance)/(1225*scalar))*2*tan(deg_rad);
//    Serial.print(y_motion);
//    Serial.print(",");
    Serial.println(y_motion_scaled);
    //Serial.print(",");
    //Serial.println(surface_quality);
  }

  if (newPulseDurationAvailable) {
    newPulseDurationAvailable = false;
    unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;

    //Long Range Sensor
    Z_distance = pulseDuration * 0.0328084; //In feet
    //Serial.println(Z_distance);

    float dtFinal = micros();
    //Time between the ultrasonic measurements in seconds
    dt = (dtFinal - dtInitial) / 1000000;
    dtInitial = micros();
    newPulse = true;
    //Serial.println(dt);
  }

  //Check to see how fast it runs in Hz
  float finalTime = micros();
  float Frequency = 1000000 / (finalTime - initialTime); //In Hz
  //Serial.println(Frequency);
  initialTime = micros();
  //delay(1000);
}
