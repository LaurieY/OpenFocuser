//LYMotor using Pololu A4983 Stepper Motor Driver Carrier with Voltage Regulators 
//But based on the 
// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!


#include <avr/io.h>
#include "WProgram.h"
#include "LYMotor.h"

static uint8_t latch_state;

/**#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif
**/

/**LYMotorController::LYMotorController(void) {
}

static LYMotorController MC;

void LYMotorController::enable(void) {
#ifdef MOTORDEBUG
    //Serial.print("In Enable ");
#endif


  digitalWrite(MOTORNOTENABLE, LOW);
}


**/

/******************************************
               MOTORS
******************************************/


/******************************************
               STEPPERS
******************************************/
void LY_Stepper::init(uint16_t steps) {
 // delay(5);

	// MC.enable();
#ifdef MOTORDEBUG
 //   Serial.print("In LY_Stepper ");
#endif


 
   /* latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();*/
    
    // enable both H bridges
  /**  pinMode(11, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(11, HIGH);
    digitalWrite(3, HIGH);
	**/
	//Enable A4983
	  revsteps = steps;
  // usteps= 8;
  //steppernum = num;
 // currentstep = 0;
	
pinMode(MOTORNOTENABLE,OUTPUT);
 //  digitalWrite(MOTORNOTENABLE, HIGH);

	pinMode(MOTORSTEP,OUTPUT);
	digitalWrite(MOTORSTEP,LOW);
		pinMode(MOTORDIR,OUTPUT);
	digitalWrite(MOTORDIR,LOW);
pinMode(MOTORDIR,OUTPUT);
  	pinMode(MOTORNOTSLEEP,OUTPUT);
   digitalWrite(MOTORNOTSLEEP, HIGH);
      	pinMode(MOTORNOTRESET,OUTPUT);
   digitalWrite(MOTORNOTRESET, HIGH);
   //NOW SET Microstep to Full
   pinMode(MOTORMS1,OUTPUT);
   digitalWrite(MOTORMS1, LOW);  
  pinMode(MOTORMS2,OUTPUT);
   digitalWrite(MOTORMS2, LOW);
   pinMode(MOTORMS3,OUTPUT);
   digitalWrite(MOTORMS3, LOW);  
   digitalWrite(MOTORNOTENABLE, LOW);  
   
 //usteps= 8;
 // currentstep = 0;
 //  setSpeed(600); //DEFAULT
   setMicroSteps(8); //DEFAULT

}
LY_Stepper::LY_Stepper(uint16_t steps,uint16_t motor ) {
init(steps);
}
LY_Stepper::LY_Stepper(uint16_t steps) { // steps is basic # of full steps
init(steps); 
 }

void LY_Stepper::enable( ) { //default setup of stepper motor, sets speed to 60 and microsteps to 8
}

 void LY_Stepper::setMicroSteps(uint16_t micsteps){
 //set MS1,MS2,MS3 for the microstep values of Full, Half, Quarter,Eighth, Sixteen, given as 1,2,4,8,16
 digitalWrite(MOTORMS1, LOW);
  digitalWrite(MOTORMS2, LOW);
  digitalWrite(MOTORMS3, LOW);
  usteps= micsteps;
 switch (usteps) {
    case 1:
       break;
	   case 2:
	   digitalWrite(MOTORMS1, HIGH);
	   break;
	   case 4:
	   digitalWrite(MOTORMS2, HIGH);
	   break;
	   case 8:
	   digitalWrite(MOTORMS1, HIGH);
	   digitalWrite(MOTORMS2, HIGH);
	   break;
	   case 16:
	   digitalWrite(MOTORMS1, HIGH);
	   digitalWrite(MOTORMS2, HIGH);
	   digitalWrite(MOTORMS3, HIGH);
	   break;

 }
 }
void LY_Stepper::setSpeed(uint16_t rpm) {
unsigned long fred = (long) revsteps * (long) rpm;
Serial.print("Fred = : "); Serial.println(fred);
  usperstep = (unsigned long) 60000000 / (( long)revsteps * (long) rpm* MICROSTEP); //ignoring microstepp settings  valu is in microseconds
  Serial.print("rev step set to: "); Serial.println(revsteps, DEC);
    Serial.print("rpm set to: "); Serial.println(rpm, DEC);
  Serial.print("current step set to: "); Serial.println(usperstep);
  Serial.println("---");

//delay(3000);
  steppingcounter = 0;  // an absolute counter of position
  digitalWrite(MOTORNOTENABLE,LOW);
}

void LY_Stepper::release(void) {
 digitalWrite(MOTORNOTENABLE, HIGH);
  digitalWrite(MOTORNOTSLEEP, LOW);
 
}

void LY_Stepper::step(uint16_t steps, uint8_t dir,  uint8_t style) { // styleIS used for A4983 calls setMicroSteps
// Step number of microsteps or steps using the microstep value of 8
//Serial.print("current step STILL: "); Serial.println(usperstep);
//Serial.print("microsteps (style) : "); Serial.println(style);
//setMicroSteps(style);
//enable();
  uint32_t uspers = usperstep;
 //Serial.print("uspers step : "); Serial.println(uspers);
 //uspers=2000;
  uint8_t ret = 0;

/*if (style == INTERLEAVE) {
    uspers /= 2;
  }
 else if (style == MICROSTEP) {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;

  }
  **/
  /**********
#ifdef MOTORDEBUG
 //   Serial.print("steps = "); Serial.println(steps, DEC);
#endif
 
 ********/
 //Serial.print("stepsize = "); Serial.println(usperstep, DEC);
  while (steps--) {
    ret = onestep(dir, style);
   delayMicroseconds(uspers/2); // in micros half a cycle, the other half was inside onestep
//	delay(uspers/2000);
  //  steppingcounter += (uspers % 1000);
  //  if (steppingcounter >= 1000) {
   //   delay(1);
  //    steppingcounter -= 1000;
  //  }
  }
 
}

void LY_Stepper::step(uint16_t steps, uint8_t dir) {
uint8_t micsteps = usteps;
step(steps, dir, micsteps);
}

uint8_t LY_Stepper::onestep(uint8_t dir, uint8_t style) {  // style not used for A4983 FIXED at MICROSTEP
style = MICROSTEP;
  uint32_t uspers = usperstep;
   digitalWrite(MOTORNOTENABLE, LOW);
  digitalWrite(MOTORNOTSLEEP, HIGH);
 
 //uspers=2000;
  // next determine what sort of stepping procedure we're up to
 


    if (dir == FORWARD) {
	digitalWrite(MOTORDIR,HIGH);
      currentstep++;
    } else {
      // BACKWARDS
	  	digitalWrite(MOTORDIR,LOW);
      currentstep--;
    }

 


#ifdef MOTORDEBUG
 // Serial.print("current step time: "); Serial.println(uspers, DEC);
 //Serial.print("current delay: "); Serial.println(uspers/2000, DEC);
#endif




digitalWrite(MOTORSTEP,HIGH);
delayMicroseconds(uspers/2); // wait for half of the cycle time in microseconds so min value is 2 msecs
//	delay(uspers/2000);
digitalWrite(MOTORSTEP,LOW);


  //Serial.println(step, DEC);

  return currentstep;
}
 

