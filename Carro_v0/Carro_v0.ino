#include <Arduino.h>
#include <esp_timer.h>

esp_timer_handle_t timer;


const int DEBUG = 0;
const int delaytime = 5;
/*         PID         */
const float Kp = 2;
const float Kd = .5;
const float Ki = .2;


float P, I, D, PID;

float lsp, rsp;

float error = 0, lastError = 0;
float max_windup = 50;

void timer_callback(void* arg);
/*       END OF PID      */

/*     MOTORS       */
const int pin_MotorR_1 = 27;    
const int pin_MotorR_2 = 26;  

const int pin_MotorL_1 = 32;
const int pin_MotorL_2 = 33;

const int MotorR_1 = 0;
const int MotorR_2 = 1;

const int MotorL_1 = 2;
const int MotorL_2 = 3;

const int Motor_enable = 25;

const int resolution = 12;
const float maxpwm = 4095 - 1;

const float MAX_PERCENT_R = 100;
const float MAX_PERCENT_L = 100;

volatile float Percent_R = MAX_PERCENT_R/2;
volatile float Percent_L = MAX_PERCENT_L/2;

const float MIN_PERCENT_R = 10;
const float MIN_PERCENT_L = 10;
/*    END OF MOTORS   */


/*     IR SENSOR       */

const int IR_0 = 21;
const int IR_1 = 19;
const int IR_2 = 18;
const int IR_3 = 16;
const int IR_4 = 17;

int readings[5] = {1, 1, 0, 1, 1};

const float bigDeviation = .30;
const float smallDeviation = .2;  

/*    END OF IR SENSOR   */

/*         LDR           */

const int LDR_0 = 5;
const int LDR_1 = 2;
const int LDR_2 = 0;

const int LDR_Go = 90000;

int LDR_value[3] = {0,0,0};

/*       END OF LDR      */






/*         Function declaration         */
void set_pwm(float pwm, char motor);
int percent_to_pwm(float percent);
void read_IR();
int control_if();
void control_PID();
void calc_PID();
void go();


void setup() 
{
    // Configure the timer
  esp_timer_create_args_t timer_args = {
    .callback = &timer_callback,
    .name = "periodic_timer"
  };

  esp_timer_create(&timer_args, &timer);

  // Start the timer (10ms = 10000 microseconds)
  esp_timer_start_periodic(timer, delaytime * 1000);

  //MOTORS
  
  pinMode(pin_MotorL_1, OUTPUT);
  pinMode(pin_MotorL_2, OUTPUT);
  pinMode(pin_MotorR_1, OUTPUT);
  pinMode(pin_MotorR_2, OUTPUT);
  pinMode(Motor_enable, OUTPUT);


  ledcAttachChannel(pin_MotorR_1, 500, resolution, MotorR_1);
  ledcAttachChannel(pin_MotorR_2, 500, resolution, MotorR_2);
  ledcAttachChannel(pin_MotorL_1, 500, resolution, MotorL_1);
  ledcAttachChannel(pin_MotorL_2, 500, resolution, MotorL_2);

  //Initialize motors at 
  set_pwm(0, 'L');
  set_pwm(0, 'R');


  digitalWrite(Motor_enable, 1);

  //END OF MOTORS


  //IR SENSOR
  pinMode(IR_0, INPUT_PULLUP);
  pinMode(IR_1, INPUT_PULLUP);
  pinMode(IR_2, INPUT_PULLUP);
  pinMode(IR_3, INPUT_PULLUP);
  pinMode(IR_4, INPUT_PULLUP);

  //END OF IR SENSOR


  //LDR

  pinMode(LDR_0, OUTPUT);
  pinMode(LDR_1, INPUT);
  pinMode(LDR_2, OUTPUT);

  digitalWrite(LDR_0, 1);
  digitalWrite(LDR_0, 0);
  
  //END OF LDR


Serial.begin(115200);
}

void loop() 
{
  /*
  go();

  read_IR();

  if( control() == 0) delay(100000);

  if (DEBUG)
  {
  Serial.print("R0: ");
  Serial.print(readings[0]);
  Serial.print(" R1: ");
  Serial.print(readings[1]);
  Serial.print(" R2: ");
  Serial.print(readings[2]);
  Serial.print(" R3: ");
  Serial.print(readings[3]);
  Serial.print(" R4: ");
  Serial.println(readings[4]);
  }

  delay(delaytime);
  */



}

void timer_callback(void* arg) 
{
  read_IR();
  calc_PID();
  control_PID();
}

/*
percent 0 to 100
dir F or B
motor R or L
*/
void set_pwm(float percent, char motor)
{
  
  if (percent > MAX_PERCENT_R and motor == 'R') percent = MAX_PERCENT_R;
  if (percent > MAX_PERCENT_L and motor == 'L') percent = MAX_PERCENT_L;




  int pwm = percent_to_pwm(percent);
  
  if (DEBUG)
  {
    Serial.print("Setting");
    Serial.print(motor);
    Serial.print(" at pc: ");
    Serial.print(percent);
    Serial.print(" pwm: ");
    Serial.println(pwm);
  }


  if(motor == 'R' )
  {
    if (percent >= 0)
    {
      ledcWrite(pin_MotorR_1 , pwm);
      ledcWrite(pin_MotorR_2 , 0);  
    }

    if (percent < 0)
    {
      ledcWrite(pin_MotorR_1 , 0);
      ledcWrite(pin_MotorR_2 , pwm);  
    }
    
  }

  if(motor == 'L' )
  {
    if (percent >= 0)
    {
      ledcWrite(pin_MotorL_1 , pwm);
      ledcWrite(pin_MotorL_2 , 0);  
    }

    if (percent < 0)
    {
      ledcWrite(pin_MotorL_1 , 0);
      ledcWrite(pin_MotorL_2 , pwm);  
    }
    
  }

}

int percent_to_pwm(float percent)
{
  int pwm = percent * maxpwm / 100;
  if (DEBUG)
  {
  Serial.print("PWM: ");
  Serial.println(pwm);
  }
  return pwm;
}

void read_IR()
{

  readings[0] = digitalRead(IR_0); ;
  readings[1] = digitalRead(IR_1); ;
  readings[2] = digitalRead(IR_2); ;
  readings[3] = digitalRead(IR_3); ;
  readings[4] = digitalRead(IR_4); ;

  if(DEBUG)
  {
    Serial.print  (readings[0]);
    Serial.print  (" ");
    Serial.print  (readings[1]);
    Serial.print  (" ");
    Serial.print  (readings[2]);
    Serial.print  (" ");
    Serial.print  (readings[3]);
    Serial.print  (" ");
    Serial.println(readings[4]);
  }

}

int control_if()
{
  //Left in the black line
  //Correct to right 
  if (readings[0] == 0)
  {

    if (DEBUG) Serial.println("A lot to the left");

    if (Percent_L - bigDeviation > MIN_PERCENT_L)
    {
      Percent_L = Percent_L - bigDeviation;
      set_pwm(Percent_L, 'L');
    }
    if (Percent_R + bigDeviation < MAX_PERCENT_R)
    {
      Percent_R = Percent_R + bigDeviation;
      set_pwm(Percent_R, 'R');
    }
  }

  if (readings[1] == 0)
  {
    if (DEBUG) Serial.println("A little to the left");

    if (Percent_L - smallDeviation > MIN_PERCENT_L)
    { 
      Percent_L = Percent_L - smallDeviation;
      set_pwm(Percent_L, 'L');
    }
    if (Percent_R + smallDeviation < MAX_PERCENT_R)
    {
      Percent_R = Percent_R + smallDeviation;
      set_pwm(Percent_R, 'R');
    }
  }

  if (readings[2] == 0)
  {

    if (DEBUG) Serial.println("In the middle");

    if (Percent_R + bigDeviation< MAX_PERCENT_R)
    {
      Percent_R = Percent_R + bigDeviation;
      set_pwm(Percent_R, 'R');
    }

    if (Percent_L + bigDeviation < MAX_PERCENT_L)
    {
      Percent_L = Percent_L + bigDeviation;
      set_pwm(Percent_L, 'L');
    }
  }

  //Right in the black line
  //Correct to left

  if (readings[3] == 0)
  {

    if (DEBUG) Serial.println("A little to the right");

    if (Percent_L + smallDeviation < MAX_PERCENT_L)
    {
      Percent_L = Percent_L + smallDeviation;
      set_pwm(Percent_L, 'L');
    }
    if (Percent_R - smallDeviation > MIN_PERCENT_R)
    {
      Percent_R = Percent_R - smallDeviation;
      set_pwm(Percent_R, 'R');
    }
  }

  if (readings[4] == 0)
  {

    if (DEBUG) Serial.println("A lot to the right");

    if (Percent_L + bigDeviation < MAX_PERCENT_L)
    {
      Percent_L = Percent_L + bigDeviation;
      set_pwm(Percent_L, 'L');
    }
    if (Percent_R - bigDeviation > MIN_PERCENT_R)
    {
      Percent_R = Percent_R - bigDeviation;
      set_pwm(Percent_R, 'R');
    }
  }

  if (readings[0] == 0 and readings[1] == 0 and readings[2] == 0 and readings[3] == 0 and readings[4] == 0)
  {

    if (DEBUG) Serial.println("Stopping");

    Percent_R = 0;
    Percent_L = 0;
    set_pwm(Percent_R, 'R');
    set_pwm(Percent_L, 'L');
    return 0;
  }

  return 1;

}

void calc_PID()
{

  error = (readings[0] * (-2) + readings[1] * (-1) + readings[2] * (0) + readings[3] * (1) + readings[4] * (2));


  P = error;
  I = I + error;
  D = lastError - error;
  lastError = error;
  if(I > max_windup) I = max_windup;
  if(I < -max_windup) I = -max_windup;

  PID = P * Kp + I * Ki + D * Kd;

  if (DEBUG) 
  {
    Serial.print("P: ");
    Serial.println(P);
    Serial.print("I: ");
    Serial.println(I);
    Serial.print("D: ");
    Serial.println(D);
    Serial.print("PID: ");
    Serial.println(PID);
  }
}

void control_PID()
{
  
  if(PID > 0)
  { //Virar à esquerda
    
    Percent_L = Percent_L - PID; //motor esquerdo mais lento
    Percent_R = Percent_R + PID; //motor direito mais rápido

  } else if (PID < 0)
  { //virar à direita

    Percent_L = Percent_L - PID; //motor esquerdo mais rápido
    Percent_R = Percent_R + PID; //motor direito mais lento

  } 

  if (error == 0)
  {
    Percent_L = Percent_L + smallDeviation;
    Percent_R = Percent_R + smallDeviation;
  }


  set_pwm(Percent_L, 'L' );
  set_pwm(Percent_R, 'R' );


}

void go()
{
  int LDR_value = 0;

  while(1)
  {

    LDR_value = analogRead(LDR_1);

    if (DEBUG) 
    {
    Serial.print("LDR: ");
    Serial.println(LDR_value);
    }

    if (LDR_value > LDR_Go){
      digitalWrite(Motor_enable, 1);
      break;
    } 

  }

}
