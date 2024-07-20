/**
 *      step.h
 * 
 *      @file step.c provides the methods to control a stepper via the `AMIS-30624`
 *      @author: Stefan Heinrich, Dennis Lotz
 *      Created on: Dez. 05, 2023
 */

#ifndef STEP_H_
#define STEP_H_

#ifdef __cplusplus
extern "C" {
#endif

// ******************** Predeclare Stepper Type ********************
typedef struct Stepper Stepper_t;

// ******************** TypeDefs ********************
typedef struct {
  uint8_t value;
  char *name;
  char *description;
  uint8_t (*get)(Stepper_t*);
  void (*set)(Stepper_t*, uint8_t);
} uint8Parameter_t;

typedef struct {
  uint16_t value;
  char *name;
  char *description;
  uint16_t (*get)(Stepper_t*);
  void (*set)(Stepper_t*, uint16_t);
} uint16Parameter_t;

typedef struct {
  int16_t value;
  char *name;
  char *description;
  int16_t (*get)(Stepper_t*);
  void (*set)(Stepper_t*, int16_t);
} int16Parameter_t;


typedef struct {
  uint32_t value;
  char *name;
  char *description;
  uint32_t (*get)(Stepper_t*);
  void (*set)(Stepper_t*, uint32_t);
} uint32Parameter_t;

typedef struct {
  I2C_TypeDef *i2c;
  char *name;
  char *description;
  I2C_TypeDef* (*get)(Stepper_t*);
  void (*set)(Stepper_t*, I2C_TypeDef*);
} i2cParameter_t;

struct Stepper {	
  i2cParameter_t i2cBus;
  uint8Parameter_t i2cAddress;
  uint8Parameter_t iRun;
  uint8Parameter_t iHold;
  uint8Parameter_t vMin;
  uint8Parameter_t vMax;
  uint8Parameter_t stepMode;
  uint8Parameter_t rotationDirection;
  uint8Parameter_t acceleration; // ToDo: add name and description and setup
  uint8Parameter_t accelerationShape; // ToDo: add name and description and setup
  int16Parameter_t position; // ToDo: add name and description and setup
  uint16Parameter_t securePosition; // ToDo: add name and description and setup
  uint8Parameter_t pwmFrequency; // ToDo: add name and description and setup
  uint8Parameter_t pwmJitter; // ToDo: add name and description and setup
  uint8Parameter_t relativeMotionThreshold; // ToDo: add name and description and setup
  void (*init)(Stepper_t*, I2C_TypeDef*, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t);
  Stepper_t* (*get)(Stepper_t*);
  void (*set)(Stepper_t*, Stepper_t*);
  void (*getFullStatus1)(Stepper_t*, uint8_t[8]);
  void (*getFullStatus2)(Stepper_t*, uint8_t[8]);
  void (*setMotorParam)(Stepper_t*);
  void (*setDualPosition)(Stepper_t*, uint16_t, uint16_t);
  int16_t (*getTargetPosition)(Stepper_t*);
  uint16_t (*getRPM)(Stepper_t*);
  void (*gotoSecurePosition)(Stepper_t*);
  void (*resetPosition)(Stepper_t*);
  void (*resetToDefault)(Stepper_t*);
  void (*softStop)(Stepper_t*);
  void (*hardStop)(Stepper_t*);
};


// ******************** Stepper ********************
extern const Stepper_t stepper;


#ifdef __cplusplus
}
#endif



#endif /* STEP_H_ */
