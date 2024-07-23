/*
eRob Example:
This program is a brief example of how to use the eRob library along with a 
Finite State Machine (FSM) to implement control with a specific time and duration.
*/
#include <Arduino.h>
#include "eRob.hpp"

//Pin Definition
#define CS 22
#define INT_PIN 15
#define CAN_ID 0x01
#define CTRL_PIN 5

//Actuator Variables
float targetTorque = 0.0;
float position;
float velocity;
float torque;

//Time Variables
const long duration = 50; //in Sec
const long frecuency = 10; //in Hz
unsigned long currentMillis;
unsigned long previousMillisDuration;
unsigned long previousMillisFrecuency;
bool flag = false;

//eRob Instance
eRob erob_1(eRob::eRob_90, CAN_ID, CS, INT_PIN, SPI);
void (*interrupt_handlers[1])() = 
{
    [](){ erob_1.handleInterrupt(); }
};

//Control Function
void control()
{
  position = erob_1.getPosition();
  velocity = erob_1.getVelocity();
  torque = erob_1.getTorque();

  /*Start of Control*/
  /*End of Control*/

  if (!erob_1.setTorque(targetTorque, 2000))
  {
    Serial.println("ERROR: SET TORQUE FAILED");
  }
}

// Definition of States
enum State 
{
  STATE_IDLE,
  STATE_WAITING_INPUT,
  STATE_PROCESSING_INPUT
};

//Definition of FSM Options
enum OPTION : char
{
  OPTION_SET_CONTROL_MODE = '0',
  OPTION_TURN_ON = '1',
  OPTION_TURN_OFF = '2',
  OPTION_SET_CONTROL = '3',
  OPTION_SET_TORQUE_0 = '4',
  OPTION_HOME_POSITION = '5',
  OPTION_MOTOR_RESPONSE = '6'  
};

State currentState = STATE_IDLE;

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  pinMode(CTRL_PIN, INPUT_PULLUP);
  if (!erob_1.initialize())
  {
    Serial.println("ERROR: MCP INITIALIZE FAILED");
  }
  else
  {
    Serial.println("MCP INITIALIZE SUCCEEDED");
  }
  delay(100);
}

void loop()
{ 
  // Handling the state machine
  switch (currentState) 
  {
    case STATE_IDLE:
      Serial.println("***--- MENU INBIODROID ---***");
      Serial.print("Set Control Mode: "); Serial.println(OPTION_SET_CONTROL_MODE - '0');
      Serial.print("Turn On: "); Serial.println(OPTION_TURN_ON - '0');
      Serial.print("Turn Off: "); Serial.println(OPTION_TURN_OFF - '0');
      Serial.print("Set Control: "); Serial.println(OPTION_SET_CONTROL - '0');
      Serial.print("Set Torque 0: "); Serial.println(OPTION_SET_TORQUE_0 - '0');
      Serial.print("Home Position: "); Serial.println(OPTION_HOME_POSITION - '0');
      Serial.print("Motor Response: "); Serial.println(OPTION_MOTOR_RESPONSE - '0');
      Serial.println();
      currentState = STATE_WAITING_INPUT;
      break;

    case STATE_WAITING_INPUT:
      if (Serial.available() > 0) 
      {
        currentState = STATE_PROCESSING_INPUT;
      }
      break;

    case STATE_PROCESSING_INPUT:
      OPTION input = (OPTION)Serial.read();
      if (input == OPTION_SET_CONTROL_MODE) 
      {
        if (!erob_1.setcontrolMode())
        {
          Serial.println("ERROR: SET CONTROL MODE FAILED");
        }
        else
        {
          Serial.println("SET CONTROL MODE SUCCEEDED"); 
        }
      }

      else if (input == OPTION_TURN_ON)
      {
        if (!erob_1.turnOn())
        {
          Serial.println("ERROR: TURN ON FAILED");
        }
        else
        {
          Serial.println("TURN ON SUCCEEDED");
        }
      }

      else if (input == OPTION_TURN_OFF)
      {
        if (!erob_1.turnOff())
        {
          Serial.println("ERROR: TURN OFF FAILED");
        }
        else
        {
          Serial.println("TURN OFF SUCCEEDED");
        }
      }

      else if (input == OPTION_SET_CONTROL)
      {
        Serial.println("Inicio de Test");
        
        previousMillisDuration = millis();
        previousMillisFrecuency = millis();
        while(digitalRead(CTRL_PIN) && !flag)
        {
          currentMillis = millis();
          if (currentMillis - previousMillisDuration >= (duration * 1000))
          {
            flag = true;
          }
          if (currentMillis - previousMillisFrecuency >= (1000.0/frecuency))
          {
            previousMillisFrecuency = currentMillis;
            control();
            Serial.printf("Position: %f, Velocity: %f,  Torque: %f.\n", position, velocity, torque);
          }
        }
        flag = false;
        for (int i = 0; i < 10; i++)
        {
          if (!erob_1.setTorque(0.0, 2000))
          {
            Serial.println("ERROR: SET TORQUE 0 FAILED");
          }
          delay(100);
        }

        Serial.println("TEST SUCCESS!!");
      }

      else if (input == OPTION_SET_TORQUE_0)
      {
        if (!erob_1.setTorque(0.0, 2000))
        {     
          Serial.println("ERROR: SET TORQUE 0 FAILED");
        }
        else
        {
          Serial.println("SET TORQUE 0 SUCCEEDED");
        }
      }

      else if (input == OPTION_HOME_POSITION)
      {
        if (!erob_1.setCurrentPositionAsZero())
        {
          Serial.println("ERROR: SET CURRENT POSITION AS ZERO FAILED");
        }
        else
        {
          Serial.println("SET CURRENT POSITION AS ZERO SUCCEEDED");
        }
      }

      else if (input == OPTION_MOTOR_RESPONSE)
      {
        if (!erob_1.readFrame())
          {
            Serial.println("ERROR: READ MOTOR FAILED");
          }
      }

      else
      {
        Serial.println("ERROR: INVALID OPCION");
      }
      currentState = STATE_IDLE;
      break;
  }
}