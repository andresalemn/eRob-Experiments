/*
eRob Example:
This program is a brief example of how to use the eRob library along with a 
Finite State Machine (FSM) to implement control with a specific time and duration.
*/
#include <Arduino.h>
#include "eRob.hpp"
#include <ModbusRtu.h>
#include <SoftwareSerial.h>

//Pin Definition
#define CS 22
#define INT_PIN 15
#define CAN_ID 0x01
#define CTRL_PIN 5

// data array for modbus network sharing
uint16_t au16data[16];
SoftwareSerial mySerial(16, 17); //Create a SoftwareSerial object so that we can use software serial. Search "software serial" on Arduino.cc to find out more details.
Modbus master(0, mySerial, 5);
modbus_t telegram;
float dynamometer = 0;

//Actuator Variables
float minTorque = 0.0;  // Minimum allowable torque
float maxTorque = 20.0;  // Maximum allowable torque
float setpointTorque = 0.0;
float position;
float velocity;
float torque;
float dynamicTorqueSensor;

//Time Variables
const long duration = 30; //in Sec
const long frecuency = 10; //in Hz
unsigned long currentMillis;
unsigned long previousMillisDuration;
unsigned long previousMillisFrecuency;
bool flag = false;

#if eRobExp

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
  dynamicTorqueSensor = dynSensor();  

  /*Start of Control*/
  if (!erob_1.setTorque(setpointTorque, 2000))
  {
    Serial.println("ERROR: SET TORQUE FAILED");
  }
  /*End of Control*/
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
  OPTION_MOTOR_RESPONSE = '6',
  OPTION_CHANGE_SETPOINT = '7'  
  
};

State currentState = STATE_IDLE;

#endif eRobExp

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  pinMode(CTRL_PIN, INPUT_PULLUP);

  // Communication with the Virtual Torque Sensor
  mySerial.begin(19200);
  master.start();
  master.setTimeOut( 200 ); // if there is no answer in 2000 ms, roll over
  telegram.u8id = 1; // slave address
  telegram.u8fct = 3; // function code (this one is registers read)
  telegram.u16RegAdd = 0; // start address in slave // 0 -> Torque, 2 -> RPM
  telegram.u16CoilsNo = 2; // number of elements (coils or registers) to read
  telegram.au16reg = au16data; // pointer to a memory array in the Arduino

  #if eRobExp
  if (!erob_1.initialize())
  {
    Serial.println("ERROR: MCP INITIALIZE FAILED");
  }
  else
  {
    Serial.println("MCP INITIALIZE SUCCEEDED");
  }
  delay(100);
  #endif eRobExp
}


void loop()
{
  #if eRobExp
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
      Serial.print("Change Setpoint: "); Serial.println(OPTION_CHANGE_SETPOINT - '0');
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
            //Serial.printf("Position= %f, Velocity= %f,  Torque= %f\n", position, velocity, torque);
            Serial.printf("Setpoint= %f, Torque= %f, Torquimetro= %f\n", setpointTorque, torque, dynamicTorqueSensor);
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
        for (int i = 0; i < 100; i++)
        {
          if (!erob_1.setTorque(0.0, 2000))
          {
            Serial.println("ERROR: SET TORQUE 0 FAILED");
          }
          position = erob_1.getPosition();
          velocity = erob_1.getVelocity();
          torque = erob_1.getTorque();
            Serial.printf("Setpoint= %f, Torque= %f\n", 0, torque);
          delay(100);
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

      else if (input == OPTION_CHANGE_SETPOINT)
      {
        Serial.printf("Please enter a torque value (%f - %f)", minTorque, maxTorque);
        while (true) {
          // Check if data is available to read
          if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n'); // Read the input as a string
            input.trim(); // Remove any leading/trailing whitespace

            setpointTorque = input.toFloat();

            // Check if the input is valid and within range
            if (input == "" || (setpointTorque == 0.0 && input != "0" && input != "0.0")) {
                Serial.println("\nInvalid input. Please enter a valid number.");
            } else if (setpointTorque < minTorque || setpointTorque > maxTorque) {
                Serial.println("The value must be between " + String(minTorque) + " and " + String(maxTorque) + ". Please try again.");
            } else {
                Serial.print("\nYou have set the torque to: ");
                Serial.println(setpointTorque);
                break; // Exit the loop on successful input
            }
          }
        }
      }

      else
      {
        Serial.println("ERROR: INVALID OPCION");
      }
      currentState = STATE_IDLE;
      break;
  }

  #endif eRobExp

  #if DynTest
    master.query(telegram); // send query (only once)
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) 
    {
      dynamometer = (uint16_t(au16data[0]) << 16) | uint16_t (au16data[1]);
      Serial.println(dynamometer/10);
    }
    delay(100);
  #endif DynTest

}