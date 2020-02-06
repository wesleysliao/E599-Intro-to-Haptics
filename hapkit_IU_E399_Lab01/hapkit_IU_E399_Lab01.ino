//--------------------------------------------------------------------------
// Eatai Roth, Indiana University, Jan 2020
//
// To be used with the Stanford Hapkit.
//
// This code modifies/substitutes HapkitLab4.ino by:
// Tania Morimoto and Allison Okamura, Stanford University
// 11.16.13
// Code to test basic Hapkit functionality (sensing and force output)
//--------------------------------------------------------------------------

// Includes
#include <math.h>

// DO NOT CHANGE
// Pin declarations for Hapkit board


#define PWM_PIN 5
#define DIR_PIN 8
#define MR_PIN A2

#define SPS 500  // Samples per second, the frequency of the control loop
#define FPS 25    // FPS for visualization (simulating what will be a serial request)
// also the freq of debug message

#define PRESCALE 8
// Helper functions below:


float sign(float value) {
  if(value == 0.0)
    return 0.0;
  else if(value > 0.0)
    return 1.0;
  else
    return -1.0;
}


// CALIBRATION

int prescaleRatioTimer0 = 64 / PRESCALE; // millis(), micros(), delay() depend on Timer0. Must post-scale values to compensate for prescale.
// Timer0 default prescale is 64

unsigned long dt_FPS = prescaleRatioTimer0 * 1000000 / FPS; // microseconds between sending serial to viz
unsigned long dt = prescaleRatioTimer0 * 1000000 / SPS;     // microseconds between updating hapkit
//=========================================================
// Measured limits of MR angle sensor from 49-972 corresponds to 0-pi, 0-180
int MR_min = 49;      // minimum measurement from MR sensor
int MR_max = 976;     // maximum measurement from MR sensor
int MR_range = MR_max - MR_min;

// Measurements from devices
// Radii of pulleys in meters
double rH = 0.07;  // From pivot to middle of handle top
double rS = 0.0732; // From pivot to groove in bottom of sector pulley
double rC = 0.0047; // radius of capstan pulley

double MR2rad = 3.1415 / (MR_max - MR_min);
double MR2deg = 180.0 / (MR_max - MR_min);
//=========================================================

// Position tracking variables
int MR_0 = 0;           // current raw reading from MR sensor
int MR_1 = 0;           // last raw reading from MR sensor
int MRinit = 0;
int MRtot = 0;
int dMR = 0;

int numFlips = 0;     // keeps track of the number of flips over the 180deg mark

const int flipThresh = 512;  // threshold to determine whether or not a flip over the 180 degree mark occurred

float thetaC_0 = 0;  // Current angle of capstan (mod 180)
float thetaCtot = 0; // Cummulative angle of capstan

union {
  float asFloat;
  byte asBytes[4];
} thetaH; //Angle of handle (as a float and a byte array)

float thetaH_lowpass = 0.0;
float thetaH_lowpass_last = 0.0;

float d_thetaH = 0;
float dd_thetaH = 0;

union {
  int asInt;
  byte asBytes[4];
} F;     //Force (as a float and a byte array)

int duty = 0;       // Duty cycle

int counter = 0;    // Counter to debug timer

unsigned long lastSerialTime; // Last time we wrote a Serial message
unsigned long lastHapkitTime; // Last time we ran the control sequence

boolean DEBUG = true;

int modeNumber = 0;


//// You will write these mode functions. They will calculate force as a function of theta, dtheta, ddtheta
int mode0(float a, float da, float dda) {
   return 0;
}

int mode1(float a, float da, float dda) {
    int fo;
    if (a<15){
      fo = 0;
    }
    else{
      fo = int(60*(a-15));
    }
    return fo;
}

int mode2(float a, float da, float dda) {
  return 0;
}

int mode3(float a, float da, float dda) {
  return 0;
}

int mode4(float a, float da, float dda) {
  if( (a < 1.5) && (a > -1.5) ) {
    return int(110 * sign(a));
  } else {
    return 0;
  }
}

int mode5(float a, float da, float dda) {
  return 0;
}

int (*modeList[])(float, float, float) = {mode0, mode1, mode2, mode3, mode4, mode5};


// Filtering

float lowpass(float alpha, float measurement, float last_estimate) {

    float estimate = (alpha * measurement) + ((1.0 - alpha) * last_estimate) ;
    return estimate;
}

float discrete_derivative(float measurement, float last_measurement, float dt) {
    return ((measurement - last_measurement) / dt);
}


void setup()
{
  // Set up serial communication
  Serial.begin(230400);

  // Set PWM frequency
  setPwmFrequency(PWM_PIN, PRESCALE);
  
  // Input pins
  pinMode(MR_PIN, INPUT); // set MR sensor pin to be an input
  //DIDR0 = 0x04;   //

  // Output pins
  pinMode(PWM_PIN, OUTPUT);  // PWM pin for motor A
  pinMode(DIR_PIN, OUTPUT);  // dir pin for motor A

  // Initialize motor
  analogWrite(PWM_PIN, 0);     // set to not be spinning (0/255)
  digitalWrite(DIR_PIN, LOW);  // set direction

  // Initialize position valiables (SEE readSensor() helper function at bottom)

  // How long does it take (in microseconds) to make two sensor readings?
  lastSerialTime = micros();
  lastHapkitTime = micros();

  MRinit = readSensor();
  MR_0 = readSensor();
  MR_1 = readSensor();
}


void loop() {

  // Control sequence

  if ((micros()-lastHapkitTime) > dt) {
    lastHapkitTime = micros();
    counter+=1;

    MR_0 = readSensor();  //current raw position from MR sensor

    dMR = MR_0 - MR_1;          //difference btwn current raw position and last raw position
    MR_1 = MR_0;
    if (dMR < (-1 * flipThresh)) {
      numFlips++;
    }
    else if (dMR > flipThresh) {
      numFlips--;
    }

    MRtot = MR_0 + MR_range * numFlips - MRinit;

    thetaC_0 = float(MR_0) * MR2deg;
    thetaCtot = float(MRtot) * MR2deg;

    thetaH.asFloat = thetaCtot * rC / rS;

    thetaH_lowpass_last = thetaH_lowpass;
    thetaH_lowpass = lowpass(0.1, thetaH.asFloat, thetaH_lowpass);

    d_thetaH = discrete_derivative(thetaH_lowpass, thetaH_lowpass_last, dt);
    dd_thetaH = 0;

    F.asInt = modeList[modeNumber](thetaH.asFloat, d_thetaH, dd_thetaH);



    duty = min(abs(F.asInt), 255);

    digitalWrite(DIR_PIN, F.asInt < 0);
    analogWrite(PWM_PIN, duty);
  }

  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case 0x67:
        //F.asInt = counter;
        Serial.write(thetaH.asBytes, 4);
        Serial.write(F.asBytes, 4);
        counter = 0;
        break;
      case 0x30:
        modeNumber = 0; break;
      case 0x31:
        modeNumber = 1; break;
      case 0x32:
        modeNumber = 2; break;
      case 0x33:
        modeNumber = 3; break;
      case 0x34:
        modeNumber = 4; break;
      case 0x35:
        modeNumber = 5; break;
      default: return;
    }
  }

  // Debug or visualization Serial message
  //if (DEBUG){
  //  if ((micros()-lastSerialTime)>dt_FPS){
  //    lastSerialTime = micros();
  //    Serial.print(thetaH.asFloat);
  //    Serial.print(", ");
  //    Serial.print(duty);
  //    Serial.print(", ");
  //    Serial.println(counter);
  //    }
  //    counter = 0;
  //  }
}

//// Eatai's version, sets prescaler on Timer0, which breaks millis() and micros() so fixed with prescaleRatioTimer0 variable above.
void setPwmFrequency(int pin, int divisor) {
  byte prescale;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: prescale = 0x01; break;
      case 8: prescale = 0x02; break;
      case 64: prescale = 0x03; break;
      case 256: prescale = 0x04; break;
      case 1024: prescale = 0x05; break;
      default: return;
    }
  }

  if (pin == 5) {
    // WGM = 011 ---> Fast PWM, TOP = 255 (sets period)
    // WGM00, WGM01 are bits 0, 1 of TCCR0A, WGM01 is bit 3 of TCCR0B
    // COM0A = 10 ----> non-inverted, COM0A0, COM0A1 are bits 4,5 on TCCR0A
    // CS0 = prescale above ----> CS00, CS01, CS02 are bits 0,1,2 on TCCR0B

    TCCR0A =  _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B = TCCR0B&0b11111000|prescale;
  }
}

int readSensor() {
  return analogRead(MR_PIN) - MR_min;

}
