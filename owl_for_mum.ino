#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>



#ifndef PSTR
 #define PSTR // Make Arduino happy
#endif

#define timeSeconds 4 //time Owl is on for


//LED Colours
#define white 255
#define red 22672
#define orange 15661
#define yellow 14746
#define green 1638
#define blue 45875
#define cyan 38502
#define pink 27034
#define purple 31949

#define LED_PIN   25
#define LED_COUNT 2
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint16_t brightness = 200;
bool bl_gr = true;

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
 
int pos = 90;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 5;

//set up pwm for buzzer
int freq = 2000;
int channel = 3;
int resolution = 8;

int pos_mult = 1;

bool low = true;

// Set GPIOs for LED and PIR Motion Sensor
//const int led = 26;
const int motionSensor = 27;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
unsigned long thisTrigger = 0;
unsigned long nextTrigger = 0;
boolean startTimer = false;

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
//  Serial.println("MOTION DETECTED!!!");
  startTimer = true;
  lastTrigger = millis();
}

void start_servo() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
                                         // using SG90 servo min/max of 500us and 2400us
                                         // for MG995 large servo, use 1000us and 2000us,
                                         // which are the defaults, so this line could be
                                         // "myservo.attach(servoPin);"
  myservo.write(90);
  
}

//initialise neo pixel strip
void start_pixel(){
  strip.begin();
   strip.setBrightness(brightness);
   strip.fill(strip.gamma32(strip.ColorHSV(pink)));//strip.gamma32(strip.ColorHSV(16000)));
   strip.show();
}

//flash the eyes of the owl
void flash_colours(){
  if(bl_gr){
    strip.setPixelColor(0,strip.gamma32(strip.ColorHSV(orange)));
    strip.setPixelColor(1,strip.gamma32(strip.ColorHSV(red)));
  } else {
    strip.setPixelColor(1,strip.gamma32(strip.ColorHSV(orange)));
    strip.setPixelColor(0,strip.gamma32(strip.ColorHSV(red)));
  }
  strip.show();
  bl_gr = !bl_gr;
  
}

//play the sound to scare off the pigeons.
void play_call(){
  if(low){
    ledcWriteTone(channel, 1300); //1300
    ledcWrite(channel, 128);
  }
  else {
    ledcWriteTone(channel, 2700); //2700
    ledcWrite(channel, 128);
    
  }
  low = !low;
//  low = true;
  delay(40);
}

void move_servo(){
//  pos=90;
  myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5); 
  if (pos > 125) {
    pos_mult = -1;
  }
  if ( pos < 45) {
    pos_mult = 1;
  }
//  Serial.println(pos);
  
  pos += pos_mult*5;
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);

  //the pwm for the tone signal to the buzzer
  //buzzer connected to GPIO 12
   ledcSetup(channel, freq, resolution);
  ledcAttachPin(12, channel);

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  
  start_pixel();
  start_servo();
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
  delay(2000);
//  // Set LED to LOW
//  pinMode(led, OUTPUT);
//  digitalWrite(led, LOW);
}



void loop() {
  // Current time
 
  thisTrigger = 0;
  if(startTimer && (now - nextTrigger > (5000))) { //wait at least 5 seconds between end of last interrupt and start of next interrupt.
    play_call();
    
    if (now - thisTrigger > 1000){
      flash_colours();
      thisTrigger = millis();
      move_servo();
    }
    }
  now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    Serial.println("Motion stopped...");
//    digitalWrite(led, LOW);
    ledcWrite(channel, 0);
    strip.clear();
    strip.show();
    startTimer = false;
    nextTrigger = millis();
  }
}



















//
