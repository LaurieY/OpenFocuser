//LYMotor using Pololu A4983 Stepper Motor Driver Carrier with Voltage Regulators 
//But based on the 
// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#ifndef _LYMotor_h_
#define _LYMotor_h_

#include <inttypes.h>
#include <avr/io.h>

#define MOTORDEBUG 1

#define MICROSTEPS 8         // 8 or 16
/***
#define MOTOR12_64KHZ _BV(CS20)  // no prescale
#define MOTOR12_8KHZ _BV(CS21)   // divide by 8
#define MOTOR12_2KHZ _BV(CS21) | _BV(CS20) // divide by 32
#define MOTOR12_1KHZ _BV(CS22)  // divide by 64

#define MOTOR34_64KHZ _BV(CS00)  // no prescale
#define MOTOR34_8KHZ _BV(CS01)   // divide by 8
#define MOTOR34_1KHZ _BV(CS01) | _BV(CS00)  // divide by 64
*/
/***#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7 ***/

#define FORWARD 1
#define BACKWARD 2
//#define BRAKE 3
//#define RELEASE 4

//#define SINGLE 1
#define DOUBLE 2
//#define INTERLEAVE 3
#define MICROSTEP 8

/*
#define LATCH 4
#define LATCH_DDR DDRB
#define LATCH_PORT PORTB

#define CLK_PORT PORTD
#define CLK_DDR DDRD
#define CLK 4

#define ENABLE_PORT PORTD
#define ENABLE_DDR DDRD
#define ENABLE 7

#define SER 0
#define SER_DDR DDRB
#define SER_PORT PORTB
*/

// Arduino pin names
//#define MOTORLATCH 12
//#define MOTORCLK 8
/** OLD defs
#define MOTORDIR 11
#define MOTORNOTSLEEP 12
#define MOTORNOTRESET 10
#define MOTORNOTENABLE 9
#define MOTORSTEP 8


#define MOTORMS1 5
#define MOTORMS2 6
#define MOTORMS3 7**/
#define MOTORDIR 11
#define MOTORNOTSLEEP 9
#define MOTORNOTRESET 8
#define MOTORNOTENABLE 2
#define MOTORSTEP 10


#define MOTORMS1 3
#define MOTORMS2 4
#define MOTORMS3 5

class LYMotorController
{
  public:
    LYMotorController(void);
    void enable(void);
    friend class LY_Stepper;
    void latch_tx(void);
};

class LY_DCMotor
{
 public:
  //LY_DCMotor(uint8_t motornum, uint8_t freq =  MOTOR34_8KHZ);
  void run(uint8_t);
  void setSpeed(uint8_t);
 
 private:
  uint8_t motornum, pwmfreq;
};

class LY_Stepper {
 public:
  LY_Stepper(uint16_t steps);
    LY_Stepper(uint16_t steps,uint16_t  motor);
  void step(uint16_t , uint8_t ,  uint8_t  );
  void step(uint16_t , uint8_t ); //use predetermined microsteps value
  void setSpeed(uint16_t);
  void setMicroSteps(uint16_t);
  uint8_t onestep(uint8_t dir, uint8_t style);
  void release(void);
  void enable();

  uint16_t revsteps; // # steps per revolution
  uint8_t steppernum;

  uint32_t usperstep, steppingcounter ;
    void init(uint16_t steps);
 private:
  uint8_t currentstep , usteps;


};

uint8_t getlatchstate(void);

#endif
