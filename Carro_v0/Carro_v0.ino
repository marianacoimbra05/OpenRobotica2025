
const int DEBUG = 1;
const int delaytime = 2;

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

volatile float Percent_R = 0;
volatile float Percent_L = 0;

const float MAX_PERCENT_R = 40;
const float MAX_PERCENT_L = 42.5;

const float MIN_PERCENT_R = 10;
const float MIN_PERCENT_L = 10;
/*    END OF MOTORS   */


/*     IR SENSOR       */

const int IR_0 = 21;
const int IR_1 = 19;
const int IR_2 = 18;
const int IR_3 = 5;
const int IR_4 = 17;

int readings[5] = {1, 1, 0, 1, 1};

const float bigDeviation = .30;
const float smallDeviation = .16;  

/*    END OF IR SENSOR   */

/*         LDR           */

const int LDR_0 = 15;
const int LDR_1 = 2;
const int LDR_2 = 0;

const int LDR_Go = 900;

int LDR_value[3] = {0,0,0};

/*       END OF LDR      */



/*         Function declaration         */
void set_pwm(float pwm, char dir, char motor);
int percent_to_pwm(float percent);
int read_IR();
int control();

/*
void IRAM_ATTR isr() {
	button1.numberKeyPresses++;
	button1.pressed = true;
}
*/

bool start=true;

void setup() 
{

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
  set_pwm(MAX_PERCENT_L/2, 'F', 'L');
  set_pwm(MAX_PERCENT_R/2, 'F', 'R');

  //END OF MOTORS


  //IR SENSOR
  pinMode(IR_0, INPUT);
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);

  //END OF IR SENSOR


  //LDR

  pinMode(LDR_0, OUPUT);
  pinMode(LDR_1, INPUT);
  pinMode(LDR_2, INPUT_PULLDOWN);

  //END OF LDR


//

Serial.begin(115200);
}

void loop() 
{
  
  if(start==true){
    LDR_value[1] = analogRead(LDR_1); 
    go();
    start=false;
  }

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

}

/*
percent 0 to 100
dir F or B
motor R or L
*/
void set_pwm(float percent, char dir, char motor)
{
  
  if (percent > MAX_PERCENT_R and motor == 'R') percent = MAX_PERCENT_R;
  if (percent > MAX_PERCENT_L and motor == 'L') percent = MAX_PERCENT_L;



  int pwm = percent_to_pwm(percent);
  
  if (DEBUG)
  {
    Serial.print("Setting");
    Serial.print(motor);
    Serial.print(" to: ");
    Serial.print(dir);
    Serial.print(" at pc: ");
    Serial.print(percent);
    Serial.print(" pwm: ");
    Serial.println(pwm);
  }


  if( dir == 'F' and motor == 'R' )
  {
    
    ledcWrite(pin_MotorR_1 , pwm);
    ledcWrite(pin_MotorR_2 , 0);
  }

  if( dir == 'F' and motor == 'L' )
  {

    ledcWrite(pin_MotorL_1 , pwm);
    ledcWrite(pin_MotorL_2 , 0);
  }

  if( dir == 'B' and motor == 'R' )
  {

    ledcWrite(pin_MotorR_1 , 0);
    ledcWrite(pin_MotorR_2 , pwm);
  }

  if( dir == 'B' and motor == 'L' )
  {

    ledcWrite(pin_MotorL_1 , 0);
    ledcWrite(pin_MotorL_2 , pwm);
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

int read_IR()
{

  int reading0 = digitalRead(IR_0);
  int reading1 = digitalRead(IR_1);
  int reading2 = digitalRead(IR_2);
  int reading3 = digitalRead(IR_3);
  int reading4 = digitalRead(IR_4); 


  if (reading0 == 1 and reading1 == 1 and reading2 == 1 and reading3 == 1 and reading4 == 1) return 0;
  
  readings[0] = reading0;
  readings[1] = reading1;
  readings[2] = reading2;
  readings[3] = reading3;
  readings[4] = reading4;

}



int control()
{
  //Left in the black line
  //Correct to right 
  if (readings[0] == 0)
  {

    if (DEBUG) Serial.println("A lot to the left");

    if (Percent_L - bigDeviation > MIN_PERCENT_L)
    {
      Percent_L = Percent_L - bigDeviation;
      set_pwm(Percent_L, 'F', 'L');
    }
    if (Percent_R + bigDeviation < MAX_PERCENT_R)
    {
      Percent_R = Percent_R + bigDeviation;
      set_pwm(Percent_R, 'F', 'R');
    }
  }

  if (readings[1] == 0)
  {
    if (DEBUG) Serial.println("A little to the left");

    if (Percent_L - smallDeviation > MIN_PERCENT_L)
    { 
      Percent_L = Percent_L - smallDeviation;
      set_pwm(Percent_L, 'F', 'L');
    }
    if (Percent_R + smallDeviation < MAX_PERCENT_R)
    {
      Percent_R = Percent_R + smallDeviation;
      set_pwm(Percent_R, 'F', 'R');
    }
  }

  if (readings[2] == 0)
  {

    if (DEBUG) Serial.println("In the middle");

    if (Percent_R + bigDeviation< MAX_PERCENT_R)
    {
      Percent_R = Percent_R + bigDeviation;
      set_pwm(Percent_R, 'F', 'R');
    }

    if (Percent_L + bigDeviation < MAX_PERCENT_L)
    {
      Percent_L = Percent_L + bigDeviation;
      set_pwm(Percent_L, 'F', 'L');
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
      set_pwm(Percent_L, 'F', 'L');
    }
    if (Percent_R - smallDeviation > MIN_PERCENT_R)
    {
      Percent_R = Percent_R - smallDeviation;
      set_pwm(Percent_R, 'F', 'R');
    }
  }

  if (readings[4] == 0)
  {

    if (DEBUG) Serial.println("A lot to the right");

    if (Percent_L + bigDeviation < MAX_PERCENT_L)
    {
      Percent_L = Percent_L + bigDeviation;
      set_pwm(Percent_L, 'F', 'L');
    }
    if (Percent_R - bigDeviation > MIN_PERCENT_R)
    {
      Percent_R = Percent_R - bigDeviation;
      set_pwm(Percent_R, 'F', 'R');
    }
  }

  if (readings[0] == 0 and readings[1] == 0 and readings[2] == 0 and readings[3] == 0 and readings[4] == 0)
  {

    if (DEBUG) Serial.println("Stopping");

    Percent_R = 0;
    Percent_L = 0;
    set_pwm(Percent_R, 'F', 'R');
    set_pwm(Percent_L, 'F', 'L');
    return 0;
  }

  return 1;

}

void go()
{
  if (LDR_value[1] > LDR_Go){
    digitalWrite(Motor_enable, HIGH);
  }

}
