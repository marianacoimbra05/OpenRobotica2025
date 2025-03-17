#include <dummy.h>


const int ReadPin=14;
const int WritePin=12;

const int pin_MotorR_1 = 16;    
const int pin_MotorR_2 = 17;  

const int pin_MotorL_1 = 18;
const int pin_MotorL_2 = 19;

const int MotorR_1 = 0;
const int MotorR_2 = 1;

const int MotorL_1 = 2;
const int MotorL_2 = 3;

void set_pwm(int pwm, char dir, char motor);

//Interrupts
struct Button {
	const uint8_t PIN;
	uint32_t numberKeyPresses;
	bool pressed;
};

Button button1 = {18, 0, false};

void IRAM_ATTR isr() {
	button1.numberKeyPresses++;
	button1.pressed = true;
}


void setup() {
  // put your setup code here, to run once:
  ledcSetup(MotorR_1, 2000, 10);
  ledcAttachPin(pin_MotorR_1, MotorR_1);
  
  ledcSetup(MotorR_2, 2000, 10);
  ledcAttachPin(pin_MotorR_2, MotorR_2);

  ledcSetup(MotorL_1, 2000, 10);
  ledcAttachPin(pin_MotorL_1, MotorL_1);

  ledcSetup(MotorL_2, 2000, 10);
  ledcAttachPin(pin_MotorL_2, MotorL_2);

  Serial.begin(115200);

  //initialize motors at 0
  set_pwm(0, 'F', 'L');
  set_pwm(0, 'F', 'R');



//Interrupts
pinMode(button1.PIN, INPUT_PULLUP);
attachInterrupt(button1.PIN, isr, FALLING);
//
}

void loop() {
  // put your main code here, to run repeatedly:

/*  
  digitalWrite(WritePin, HIGH);
  //void analogWrite(2, 180);
  int a=digitalRead(ReadPin);

  if (a==1){
    Serial.println("Hello World");
  }
  else 
    Serial.println("Not Hello World");
  a=0;
*/

  set_pwm(0, 'F', 'L');
  set_pwm(0, 'F', 'R');


  delay(5000);

  set_pwm(1023, 'F', 'L');
  set_pwm(1023, 'F', 'R');

  delay(5000);

  set_pwm(0, 'B', 'L');
  set_pwm(0, 'B', 'R');

  delay(5000);

  set_pwm(1023, 'B', 'L');
  set_pwm(1023, 'B', 'R');

  delay(5000);

  //void analogWriteResolution(uint8_t pin, uint8_t resolution);
  //void analogWriteFrequency(uint8_t pin, uint32_t freq);
/*
	if (button1.pressed) {
		Serial.printf("Button has been pressed %u times\n", button1.numberKeyPresses);
		button1.pressed = false;
	}
*/
}


void set_pwm(int pwm, char dir, char motor){
  //pwm 0 a 255
  //dir F or B
  //motor R or L

  if( dir == 'F' and motor == 'R' )
  {
    ledcWrite(MotorR_1 , pwm);
    ledcWrite(MotorR_2 , 0);
  }

  if( dir == 'F' and motor == 'L' )
  {
    ledcWrite(MotorL_1 , pwm);
    ledcWrite(MotorL_2 , 0);
  }

  if( dir == 'B' and motor == 'R' )
  {
    ledcWrite(MotorR_1 , 0);
    ledcWrite(MotorR_2 , pwm);
  }

  if( dir == 'B' and motor == 'L' )
  {
    ledcWrite(MotorL_1 , 0);
    ledcWrite(MotorL_2 , pwm);
  }
}
