//#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
#include <math.h> 
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPIN 13

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed creating more than
// 40000 encoder ticks per second per motor.
 
// Quadrature encoders
#define encoderPinA 2
#define encoderPinB 4
#define isrPin 5

int enA = 6;
int in1 = 8;
int in2 = 9;

//  encoder
//#define c_EncoderInterrupt 3
//#define c_EncoderPinA 3
//#define c_EncoderPinB 5
volatile bool _EncoderBSet;
volatile long _EncoderTicks = 0;

int nISR, stop_n, n_delay;

#define offset 0

int inByte;
int outByte;

int fresh;
bool send_ser;

float amp;
float freq;
float w;
float theta_d;
float dt;
float dt2;
float kp;
int pwm;
int width;
int curspeed;
int e;
int state;
//Servo _Servo;  // create servo object to control right motor
//Servo _LeftServo;  // create servo object to control left motor
 
//int potpin = 0;  // analog pin used to connect the potentiometer
//int val;    // variable to read the value from the analog pin


//int pwmA = 6;//3           // the pin that the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
 
//unsigned char MSB = 0;  // 1 byte in Arduino
//unsigned char LSB = 0;  // 1 byte  in Arduino

unsigned long t;
unsigned long prevt;
unsigned long t0;
unsigned long tempt;
float tprint;
float t_sec;
int mydelay;
int prevenc;
int posbreak;
int negbreak;
int test_case;

int preve;
float edot;

void setup()
{
  Serial.begin(115200);
  //Serial.print("Lab 8 - frequency response");
  Serial.print("Dr. Krauss wuz here");
  Serial.print("\n");

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

    //---------------------------
  //
  // Timer Interrupt stuff
  //
  //---------------------------
  pinMode(LEDPIN, OUTPUT);
  // initialize Timer1

  // Left encoder
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  pinMode(isrPin, OUTPUT);
  digitalWrite(isrPin, LOW);
  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoder, RISING);

  amp = 0;
  width = 0;
  stop_n = 1000;
  nISR = stop_n + 10;
  n_delay = 10;
  dt = 1.0/250;
  kp = 3;
  state = 0;
  posbreak = 60;// <-- you may want to adjust this
  negbreak = -60;// <-- this too
}

int inv_deadband(int speed){
  int out;
  out = speed;
  
  if ( out > 0){
    out += posbreak;
  }
  else if (out < 0){
    out += negbreak;
  }
  
  if (out < -255){
    out = -255;
  }
  else if (out > 255){
    out = 255;
  }

  return out;
}
void command_motor(int speed){

  speed = inv_deadband(speed);

  if (speed > 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, speed);
  }
  else if (speed < 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, abs(speed));
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}

void prepsine(){
    Serial.println("Input sine amp:");
    while (Serial.available() == 0){
	delay(10);
      }
    amp = Serial.parseFloat();

    Serial.println("Input sine freq (Hz):");
    while (Serial.available() == 0){
	delay(10);
      }
    freq = Serial.parseFloat();
    w = 2*3.1415926*freq/1000;// account for milliseconds
    Serial.print("#amp = ");
    Serial.println(amp);
    Serial.print("#freq = ");
    Serial.println(freq);
    Serial.print("#w = ");
    Serial.println(w);
}

void prepstep(){
    Serial.println("Input step amp:");
    while (Serial.available() == 0){
	delay(10);
      }
    amp = Serial.parseInt();
    Serial.println("Input Kp:");
    while (Serial.available() == 0){
	delay(10);
      }
    kp = Serial.parseInt();
}

void preptest(){
    Serial.println("Input test:");
    Serial.println("case (1) - fixed sine");
    Serial.println("case (2) - step");
    while (Serial.available() == 0){
	delay(10);
      }
    test_case = Serial.parseInt();

    if ( test_case == 1){
      prepsine();
    }
    else if (test_case == 2){
      prepstep();
    }
    else{
      Serial.println("invalid case:");
      Serial.println(test_case);
    }
    
    _EncoderTicks = 0;
    Serial.println("#nISR,dt (ms),t (ms),theta_d,encoder,pwm");
    delay(50);
}

void loop()
{
  if ( state == 0){
      preptest();
      state = 1;
      nISR = 0;
      t0 = micros();
      prevt = t0;
      t = t0;
  }
    
  else if (state == 1){
    if (nISR < (stop_n-100)){
    prevt = t;
    t = micros();
    nISR++;
    dt = (t-prevt)/1000.0;
    tprint = (t-t0)/1000.0;
    if ( test_case == 1){
      theta_d = (int)(amp*sin(w*tprint));
    }
    else if (test_case == 2){
      theta_d = (int)(amp);
    }
    else{
      theta_d = 0;
    }
    e = theta_d - _EncoderTicks;
    pwm = (int)(kp*e);
    if ( nISR < n_delay){
	curspeed = 0;
    }
    else {
      command_motor(pwm);
      curspeed = pwm;
    }
    Serial.print(nISR);
    Serial.print(",");    
    Serial.print(dt);
    Serial.print(",");    
    Serial.print(tprint);
    Serial.print(",");    
    Serial.print(theta_d);
    Serial.print(",");    
    Serial.print(_EncoderTicks);
    Serial.print(",");    
    Serial.print(pwm);
    tempt = micros();
    dt2 = (tempt-t)/1000;
    Serial.print('\n');
    mydelay = 2900 + t - tempt;
    delayMicroseconds(mydelay);
    }
 
  else {
    Serial.println("end");
    Serial.println("");
    command_motor(0);
    state = 0;
   }
  }
}
 
// Interrupt service routines for the right motor's quadrature encoder
void doEncoder()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  //n++;

  _EncoderBSet = digitalRead(encoderPinB);   // read the input pin
  
  // and adjust counter + if A leads B
  if (_EncoderBSet){
    _EncoderTicks ++;
  }
  else {
    _EncoderTicks --;
  }
}


