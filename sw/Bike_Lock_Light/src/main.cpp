#include <Arduino.h>

#include <FastLED.h>

#include <HardwareBLESerial.h>
#include<Wire.h>
#include<ADXL345_WE.h>
#define ADXL345_I2CADDR 0x53 // 0x1D if SDO = HIGH

HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();
ADXL345_WE myAcc = ADXL345_WE(ADXL345_I2CADDR);

#define LED_PIN     D5                //Required for Ardunino Nano ESP32
#define SENSOR_PIN  A0                //interface pin with magnetic sensor
#define ACC_PIN      0                //pin for accelerometer
#define NUM_LEDS    20
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define RED 0
#define YELLOW 1
#define UPDATES_PER_SECOND 20

#define PANIC_G_THRESHOLD 2.0
#define PANIC_TIMEOUT 5000

const int LED1    =   5;
const int LED2    =   4;
const int LED3    =   3;

int val;              //variable to store read values
int acc_val;          //variable to store values read from accelerometer
CRGB leds[NUM_LEDS];  //array of led colors to be displayed at each spot
CRGBPalette16 currentPalette;   //current set of colors being used
TBlendType    currentBlending;  //current blending scheme

void setup() {
  delay(3000);  // power-up safety delay
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(A0, INPUT);   // set analog pin as input
  pinMode(D8, INPUT);   // set button as input
  pinMode(D9, INPUT);   // set button as input
  Serial.begin(115200);   // initialize serial interface

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  currentBlending = NOBLEND;
  fill_solid(currentPalette, 16, CRGB::Black);  //led strip initalized to black

  // Bluetooth
  bleSerial.beginAndSetupBLE("Bike Lock");

  // Accelerometer
  Wire.begin();
  Serial.println("ADXL345_Sketch - Basic Data");
  Serial.println();
  if(!myAcc.init()){
    Serial.println("ADXL345 not connected!");
  }

  myAcc.setDataRate(ADXL345_DATA_RATE_50);
  delay(100);
  Serial.print("Data rate: ");
  Serial.print(myAcc.getDataRateAsString());

  myAcc.setRange(ADXL345_RANGE_4G);
  Serial.print("  /  g-Range: ");
  Serial.println(myAcc.getRangeAsString());
  Serial.println();
}

uint flash = 0;

bool right_last = false;
bool left_last = false;

bool right_anim = false;
bool left_anim = false;

bool panic = false;

void bluetoothUpdate(bool panic){
  bleSerial.println(panic ? "Panic!" : "Ending panic");
}

void checkSensors(){
  // check accelerometer
  xyzFloat g;
  myAcc.getGValues(&g);

  static int panicTime = 0;
  if (abs(g.x) + abs(g.y) + abs(g.z) > PANIC_G_THRESHOLD){
    panic = true;
    panicTime = millis();
    bluetoothUpdate(true);
  }

  if (panic && (millis() - panicTime) > PANIC_TIMEOUT){
    panic = false;
    bluetoothUpdate(false);
  }

  // check buttons
  bool right = !digitalRead(D8);
  bool left = !digitalRead(D9);

  if (right && !right_last){
    // toggle blinker anim
    right_anim = !right_anim;
  }

  if (left && !left_last){
    // toggle blinker anim
    left_anim = !left_anim;
  }

  right_last = right;
  left_last = left;
}

void ledUpdate(){

  fill_solid(leds, NUM_LEDS, CRGB::Black);

  if (panic){
    if (flash >= 8){
      fill_solid(leds, NUM_LEDS, CRGB::Red);
    }
  } else {
    // left blinker
    if (left_anim && flash >= 8){
      //right blinker on
      fill_solid(&leds[NUM_LEDS - 3], 3, 0xFFBF00); // Amber
      digitalWrite(LED2, LOW);
    } else {
      digitalWrite(LED2, HIGH);
    }

    // right blinker
    if (right_anim && flash >= 8){
      //right blinker on
      fill_solid(leds, 3, 0xFFBF00); // Amber
      digitalWrite(LED3, LOW);
    } else {
      digitalWrite(LED3, HIGH);
    }

    // Middle bar
    static int counter = 3;
    static bool reverse = false;
    Serial.println(counter);
    fill_solid(&leds[counter], 2, CRGB::Red);
    if (!reverse && counter == NUM_LEDS - 5){
      reverse = true;
    } else if (reverse && counter == 3){
      reverse = false;
    }
    counter = reverse ? (counter - 1) : (counter + 1); 
    

  }

  flash = (flash + 1) % 16;
    
  FastLED.show();
}

void loop() {
  //val = analogRead(SENSOR_PIN);  //read sensor value
  //acc_val = analogRead(ACC_PIN); //read accelerometer value

  checkSensors();
  ledUpdate();
  bleSerial.poll();

  //Serial.println("Update!");
  
  FastLED.delay(1000 / UPDATES_PER_SECOND);
}
