#include <math.h> 
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPIN 13

// Encoder pins
// Verify that these are the same pins your encoder
// A and B channels are connected to
#define encoderPinA 2
#define encoderPinB 3

// PWM and enable pins for the L298 H bridge
// Verify that these are the same pins your
// H bridge is connected to
int pwn_pin = 5;
int in1 = 6;
int in2 = 7;

//  encoder
volatile bool _EncoderBSet;
volatile long encoder_count = 0;

int n_loop, stop_n, n_delay;

#define offset 0

int inByte;
int outByte;

float amp;
float freq;
float w;
float theta_d;
float dt_ms;
float dt_sec;
float kp;
float kd;
int pwm;
int width;
int curspeed;
int e;
int state;
int test_case;

unsigned long t;
unsigned long prevt;
unsigned long t0;
unsigned long tempt;
float t_ms;
float t_sec;
int mydelay;
int prevenc;
int posbreak;
int negbreak;

int preve;
float edot;

void setup()
{
  Serial.begin(115200);
  Serial.print("Root Locus Lab");
  Serial.print("\n");

  pinMode(pwn_pin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(LEDPIN, OUTPUT);

  // encoder
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoder, RISING);

  n_loop = 1000;
  amp = 0;
  width = 0;
  stop_n = 500;
  n_delay = 10;
  kp = 3;
  kd = 0;
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
    analogWrite(pwn_pin, speed);
  }
  else if (speed < 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwn_pin, abs(speed));
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwn_pin, 0);
  }
}

void calibrate_deadband(){
  posbreak = 0;
  negbreak = 0;
  prevenc = encoder_count;
  for (int a=0; a<200; a++){
    command_motor(a);
    delay(50);
    if (encoder_count != prevenc){
      Serial.print("posbreak amp = ");
      Serial.println(a);
      posbreak = a;
      break;
    }
  }
  command_motor(0);
  delay(100);
  prevenc = encoder_count;
  for (int a=0; a>-200; a--){
    command_motor(a);
    delay(50);
    if (encoder_count != prevenc){
      Serial.print("negbreak amp = ");
      Serial.println(a);
      negbreak = a;
      break;
    }
  }
  command_motor(0);
}

void preptest(){
  Serial.println("#====================");
  Serial.println("#n_loop,dt (ms),t (ms),theta_d,encoder,pwm");
  encoder_count = 0;
  n_loop = 0;
  t0 = micros();
  prevt = t0;
  t = t0;
}

int get_int(){
  int out_int;

  while (Serial.available() == 0){
    delay(10);
  }
  out_int = Serial.parseInt();
  return(out_int);
  
}

float get_float(){
  float out_float;

  while (Serial.available() == 0){
    delay(10);
  }
  out_float = Serial.parseFloat();
  return(out_float);
  
}

void menu(){
  Serial.println("Case 1: calibrate inverse deadband");
  Serial.println("Case 2: open-loop pulse test");
  Serial.println("Case 3: P control step response test");
  Serial.println("Case 4: PD step response test");

  test_case = 0;// default value if not over written below
  state = 0;// If something goes wrong, set code to return to menu
    
  while (Serial.available() == 0){
    delay(10);
  }
  inByte = Serial.read();
  if ( inByte == '1' ){
    Serial.println("Calibrating...");
    state = 2;
  }
  else if (inByte == '2'){
    Serial.println("Open-loop pulse test");
    Serial.println("Input the pulse amplitude (pwm counts)");
    amp = get_int();
    Serial.println("Input the pulse width (loop counts)");
    width = get_int();
    Serial.print("#amp = ");
    Serial.print(amp);
    Serial.print('\n');
    Serial.print("#width = ");
    Serial.print(width);
    Serial.print('\n');
    preptest();
    test_case = 2;
    state = 1;
  }
  else if (inByte == '3'){
    Serial.println("P control step response test");
    Serial.println("Input the step amplitude (encoder counts)");
    amp = get_int();
    Serial.println("Input Kp");
    kp = get_float();
    Serial.print("#amp = ");
    Serial.print(amp);
    Serial.print('\n');
    Serial.print("#kp = ");
    Serial.print(kp);
    Serial.print('\n');
    preptest();
    test_case = 3;
    state = 1;
  }    
  else if (inByte == '4'){
    Serial.println("PD step response test");
    Serial.println("Input the step amplitude (encoder counts)");
    amp = get_int();
    Serial.println("Input Kp");
    kp = get_float();
    Serial.println("Input Kd");
    kd = get_float();
    Serial.print("#amp = ");
    Serial.print(amp);
    Serial.print('\n');
    Serial.print("#kp = ");
    Serial.print(kp);
    Serial.print('\n');
    Serial.print("#kd = ");
    Serial.print(kd);
    Serial.print('\n');
    preptest();
    test_case = 4;
    state = 1;
  }    
  else{
    Serial.println("Case not recognized");
  }
}

void loop()
{
  if ( state == 0){
    menu();
  }
    
  else if (state == 1){
    //main loop for running OL pulse, P control step, or PD step responses
    if (n_loop < (stop_n)){
      prevt = t;
      t = micros();
      n_loop++;
      dt_ms = (t-prevt)/1000.0;
      dt_sec = dt_ms/1000.0;
      t_ms = (t-t0)/1000.0;
      t_sec = t_ms/1000.0;
    
      if ( n_loop < n_delay ){
	// all cases start with a little bit of "off" time to
	// establish initial conditions
	pwm = 0;
	theta_d = 0;
      }
      else if ( n_loop < stop_n ){
	// determine pwm of the different test cases:
	// OL pulse, Kp step, and PD step
	if ( test_case == 2){
	  // OL pulse test
	  // set pwm equal to zero or amp depending on whether or not
	  // we are in the "on" or "off" parts of the test

	  theta_d = 0;// not relevant in open loop
	  
	  if (n_loop < (n_delay + width)){
	    pwm = amp;
	  }
	  else{
	    pwm = 0;
	  }
	}
	else if (test_case > 2){
	  theta_d = amp;
	  preve = e;
	  e = theta_d - encoder_count;
	  edot = ((float)(e-preve))/dt_sec;
	  if ( test_case == 3 ){
	    // P control
	    pwm = (int)(kp*e);
	  }
	  else if (test_case == 4){
	    // PD control
	    pwm = (int)(kp*e + kd*edot);
	  }
	  else{
	    pwm = 0;//something is wrong if we end up here
	  }
	}
	else{
	  // something is wrong if test case is not one of 2, 3, or 4
	  pwm = 0;
	}
      }

      command_motor(pwm);

      // print data to serial monitor
      Serial.print(n_loop);
      Serial.print(",");    
      Serial.print(dt_ms);
      Serial.print(",");    
      Serial.print(t_sec);
      Serial.print(",");    
      Serial.print(theta_d);
      Serial.print(",");    
      Serial.print(encoder_count);
      Serial.print(",");    
      Serial.print(pwm);
      tempt = micros();
      Serial.print('\n');
      mydelay = 3900 + t - tempt;
      delayMicroseconds(mydelay);
    }
 
    else {
      Serial.println("#end");
      Serial.println("#====================");
      Serial.println("");
      command_motor(0);
      state = 0;
    }
  }
  else if ( state == 2){
    Serial.println("in calibration");
    calibrate_deadband();
    state = 0;
  }
  //delay(10);
}
 
// Interrupt service routines for the right motor's quadrature encoder
void doEncoder()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  //n++;

  _EncoderBSet = digitalRead(encoderPinB);   // read the input pin
  
  // and adjust counter + if A leads B
  if (_EncoderBSet){
    encoder_count --;
  }
  else {
    encoder_count ++;
  }
}
