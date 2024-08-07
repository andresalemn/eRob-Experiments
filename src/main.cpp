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
#define BUTTON_PIN 12
#define RX2_PIN 16
#define TX2_PIN 17
#define MODBUS_MODE_PIN 13

#if eRobExp
  //Actuator Variables
  float position;
  float degPosition;
  float velocity;
  float degVelocity;
  float torque;
  float minTorque = -10.0;  // Minimum allowable torque
  float maxTorque = 10.0;   // Maximum allowable torque
  float sat = 10.0;         // Motor's torque saturation
  float setpointTorque = -10.0;
  float dynamicTorqueSensor;

  //Time Variables
  float duration = 5.0; //in Sec
  const long frecuency = 10; //in Hz
  const long printingFrecuency = 100; //in ms
  unsigned long currentMillis;
  unsigned long previousMillisDuration;
  unsigned long previousMillisFrecuency;
  unsigned long previousMillisPrintingFrecuency;
  bool flag = false;

  /* Control Variables*/
  byte cmd = 0;  // Un byte que utilizamos para la comunicación serie (cmd=comando)
  
  // Define the boolean variable and a previous state variable
  bool currentStateKd = false;
  bool previousStateKd = false;

  // Define variables for debouncing
  volatile unsigned long lastDebounceTimeKd = 0;
  const unsigned long debounceDelayKd = 1000; // 1000 milliseconds debounce delay

  unsigned long previousMillisKd = 0;
  unsigned long currentMillisKd;
  const long intervalKd = 100; // Adjust the interval as needed

  // Return to Zero Mode
  float posError;
  float degposError = 0.0;
  float derivativeError;
  float previousError = 0.0; // Store the previous error
  float targetPos = 0.0;
  float Kp = 20.0; // Proportional gain, adjust as needed
  float Kd = 5.0; // Derivative gain, adjust as needed
  float Ke = 1.0; // Exponential term for the PD Control
  float toleranceZero = 0.5; // Define a small tolerance for zero position
  unsigned long currentZeroMillis;
  unsigned long previousZeroMillis = 0;
  const long intervalZero = 50; // Check every 50 milliseconds
#endif

#if DynTest //Usar el dinamometro externo
  // data array for modbus network sharing
  uint16_t au16data[16];    //Max Lenght Buffer Array
  SoftwareSerial mySerial(RX2_PIN, TX2_PIN); //Create a SoftwareSerial object so that we can use software serial. Search "software serial" on Arduino.cc to find out more details.
  Modbus master(0, mySerial, MODBUS_MODE_PIN);   //Last parameter 
  modbus_t telegram;
  float dynamometer = 0;

  float dynSensor()
  {
    master.query(telegram); // send query (only once)
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) 
    {
      dynamometer = (uint16_t(au16data[0]) << 16) | uint16_t (au16data[1]);
      return (dynamometer/10);
    }
    return (dynamometer/10);
  }

#endif

#if rampTest //Ramp variables
float startRampValue = 0.0; // Initial value of the ramp
float finalRampValue = 10.0; // Final peak value
float incrementRamp = 0.1; // Increment step size
float decrementRamp = 0.0; // Decrement step size
float currentRampValue; // Current value of the ramp
bool isRamping = false; // State of the ramp function
bool rampCompleted = false; // Flag to indicate if ramp is completed
unsigned long previousRampMillis = 0; // Store the last time the ramp value was updated
const unsigned long intervalRamp = 100; // Interval between updates in milliseconds

// Function to start the ramp
void startRamp() {
  isRamping = true;
  rampCompleted = false;
  currentRampValue = startRampValue; // Reset the current value
  //Serial.println("Ramp function started.");
}

// Function to run the ramp
void runRamp() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousRampMillis >= intervalRamp) {
    previousRampMillis = currentMillis;

    // Increment phase
    currentRampValue += incrementRamp;
    //Serial.println(currentRampValue); // Print value after increment
    setpointTorque = currentRampValue;

    // Decrement phase
    currentRampValue -= decrementRamp;
    //Serial.println(currentRampValue); // Print value after decrement
    setpointTorque = currentRampValue;

    // Check if final value reached and stop ramp if so
    if (currentRampValue >= finalRampValue) {
      currentRampValue = finalRampValue;
      //Serial.println(currentRampValue); // Print final value
      isRamping = false; // Stop the ramp function
      rampCompleted = true; // Mark the ramp as completed
      //Serial.println("Ramp function completed.");
    }

    // Reset values if ramp is completed
    if (rampCompleted) {
      // Optionally reset any other state variables or perform cleanup here
      isRamping = false;
      rampCompleted = false;
    }
  }
}
#endif

#if eRobExp
  //eRob Instance
  eRob erob_1(eRob::eRob_90, CAN_ID, CS, INT_PIN, SPI);
  void (*interrupt_handlers[1])() = 
  {
      [](){ erob_1.handleInterrupt(); }
  };
 
  void IRAM_ATTR toggleBoolean() {
    currentMillisKd = millis();
    
    // Check if the time since the last interrupt is greater than the debounce delay
    if ((currentMillisKd - lastDebounceTimeKd) > debounceDelayKd) {
      currentStateKd = !currentStateKd;
      lastDebounceTimeKd = currentMillisKd;
    }
  }

float Sign(float number) // Sign function
{
    if (number < 0)
    {
        return -1.0;
    }
    else
    {
        return 1.0;
    }
}

  void updateKd() // Function to update the kd condition
  {
    // Get the current time
    currentMillisKd = millis();

    // Check if the interval has passed
    if (currentMillisKd - previousMillisKd >= intervalKd) {
      // Update the previousMillisKd to the current time
      previousMillisKd = currentMillisKd;

      // Check if the state has changed
      if (currentStateKd != previousStateKd) {
        // Print different messages based on the state
        if (currentStateKd) {
          Serial.println("The boolean variable has changed to true!");
        } else {
          Serial.println("The boolean variable has changed to false!");
        }
        // Update the previous state to the current state
        previousStateKd = currentStateKd;
      }
    }
  }

  void getData()  // Function to obtain the actual values of position, velocity and torque
  {
    position = erob_1.getPosition();
    degPosition = position* (180.0 / PI);
    degposError = targetPos - degPosition;
    velocity = erob_1.getVelocity();
    degVelocity = velocity* (180.0 / PI);
    torque = erob_1.getTorque();
  }

  void control()  // Control Function
  {
    getData();

    #if DynTest //Usar el dinamometro externo
    dynamicTorqueSensor = dynSensor();
    /*Start of Control*/
    if (!erob_1.setTorque(setpointTorque, 2000))
    {
      Serial.println("ERROR: SET TORQUE FAILED");
    }
    /*End of Control*/
    #endif

    #if normalTest
      /*Start of Control*/
      if (!erob_1.setTorque(setpointTorque, 2000))
      {
        Serial.println("ERROR: SET TORQUE FAILED");
      }
      /*End of Control*/
    #endif

    #if buttonTest
      if (currentStateKd == true)
      {
        setpointTorque = (-Kd)*velocity;
        // setpointTorque = 0.0;
      } 

      if (currentStateKd == false)
      {
        setpointTorque = -10.0;
      } 

      if (!erob_1.setTorque(setpointTorque, 2000))
      {
        Serial.println("ERROR: SET TORQUE FAILED");
      }
    #endif

    #if rampTest
      /*Start of Control*/
      if (!erob_1.setTorque(setpointTorque, 2000))
      {
        Serial.println("ERROR: SET TORQUE FAILED");
      }
      /*End of Control*/
      // Update the current ramp value based on the state
      if (isRamping) {
        runRamp();
      }
    #endif

    #if positionTest
      /*Start of Control*/
      if (!erob_1.setTorque(setpointTorque, 2000))
      {
        Serial.println("ERROR: SET TORQUE FAILED");
      }
      /*End of Control*/
    #endif
  }

  void goToZero() // Function that returns the shaft to its HOME (0°)
  {  
    unsigned long currentZeroMillis = millis();

    getData();

    if (currentZeroMillis - previousZeroMillis >= intervalZero) {
        previousZeroMillis = currentZeroMillis;
        
        // Get the current position and velocity
        position = erob_1.getPosition();
        degPosition = position * (180.0 / PI);
          velocity = erob_1.getVelocity();
        
        // Check if we are within the tolerance range
        if (abs(degPosition) > toleranceZero) {

            // Calculate the error
            // posError = targetPos - position; // When target is 0, error is -position
            // Calculate the derivative of the error
            // derivativeError = (posError - previousError) / intervalZero;

            // Calculate the torque using the PD controller
            // setpointTorque = Kp * posError + Kd * derivativeError;
            setpointTorque = - Kp * (position - targetPos) - Kd * velocity - Ke * Sign(position - targetPos) * pow(fabs(position - targetPos), 0.5);

            
            // Apply torque limits
            // if (setpointTorque > maxTorque) {
            //     setpointTorque = maxTorque;
            // } else if (setpointTorque < minTorque) {
            //     setpointTorque = minTorque;
            // }

            // Motors torque saturation
            if (fabs(setpointTorque) > sat)
            {
              setpointTorque = Sign(setpointTorque) * sat;
            }            

            // Apply torque to the motor
            if (!erob_1.setTorque(setpointTorque, 2000)) {
                Serial.println("ERROR: SET TORQUE FAILED");
            }
        }
    }
  }

  void printData()  // Function to print the data to the serial monitor depending on the testing mode
  {
    #if normalTest
      //Serial.printf("Position= %f, Velocity= %f,  Torque= %f\n", position, velocity, torque);
      Serial.printf("Setpoint= %f, Torque= %f, Position= %f, Velocidad= %f\n", setpointTorque, torque, degPosition, degVelocity);
    #endif

    #if DynTest //Usar el dinamometro externo
      Serial.printf("Setpoint= %f, Torque= %f, Torquimetro= %f\n", setpointTorque, torque, dynamicTorqueSensor);
    #endif

    #if rampTest
      // Output the current ramp value
      // Serial.printf("Setpoint= %f, Torque= %f, Torquimetro= %f\n", setpointTorque, torque, dynamicTorqueSensor);
      Serial.printf("Setpoint= %f, Torque= %f, Position= %f, Velocidad= %f\n", setpointTorque, torque, degPosition, degVelocity);
    #endif  

    #if positionTest
      Serial.printf("Setpoint= %f, Torque= %f, Position= %f\n", setpointTorque, torque, degPosition);
    #endif

    #if buttonTest
      // Serial.printf("Setpoint= %f, Torque= %f, Position= %f, Velocidad= %f\n", setpointTorque, torque, degPosition, degVelocity);
      Serial.printf("SP_Torque= %f, Torque= %f, SP_Pos= %f, Position= %f, Velocidad= %f\n", setpointTorque, torque, 0.0, degPosition, degVelocity);
      // Serial.printf("SP_Pos= %f, Position= %f, Pos_Error= %f, Velocity= %f\n", 0.0, degPosition, degposError, degVelocity);
    #endif
  }

  void printChange(byte flag){  // Imprime en el terminal serie los datos después de un cambio de variable. 
                            
    if (flag == 1) // setpointTorque changed
    {
      Serial.printf("\nYou have set the torque (Setpoint) to: %f N·m\n", setpointTorque);
    }

    if (flag == 2) // kP changed
    {
      Serial.printf("\nYou have set the Proportional Gain (Kp) to: %f\n", Kp);
    }
    
    if (flag == 3) // kD changed
    {
      Serial.printf("\nYou have set the Derivative Gain (Kd) to: %f\n", Kd);
    }
    
    if (flag == 4) // Seconds changed
    {
      Serial.printf("\nYou have set the duration of the experiment to: %f seconds\n", duration);
    }

    if (flag == 5) // Showing the current variables
    {
      Serial.println("\nThe current values are:");
      Serial.printf("Setpoint= %f, kP= %f, kD= %f, Duration= %f\n", setpointTorque, Kp, Kd, duration);
    }
  }

  enum State // Definition of States of the menu
  {
    STATE_IDLE,
    STATE_WAITING_INPUT,
    STATE_PROCESSING_INPUT
  };

  enum OPTION : char //Definition of Finite-state machine (FSM) Options
  {
    OPTION_SET_CONTROL_MODE = '0',
    OPTION_TURN_ON = '1',
    OPTION_TURN_OFF = '2',
    OPTION_SET_CONTROL = '3',
    OPTION_SET_TORQUE_0 = '4',
    OPTION_HOME_POSITION = '5',
    OPTION_MOTOR_RESPONSE = '6',
    OPTION_CHANGE_VALUES = '7',  
    OPTION_GO_ZERO = '8' 
  };

  State currentState = STATE_IDLE;

#endif

void setup()
{
  Serial.begin(115200);
  SPI.begin();

  // Configure the pin as an input with internal pull-up resistor
  pinMode(CTRL_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), toggleBoolean, FALLING); // Configure the interrupt

  #if DynTest //Usar el dinamometro externo
  // Communication with the Virtual Torque Sensor
  mySerial.begin(19200);
  master.start();
  master.setTimeOut(200); // if there is no answer in 2000 ms, roll over
  telegram.u8id = 1; // slave address
  telegram.u8fct = 3; // function code (this one is registers read)
  telegram.u16RegAdd = 0; // start address in slave // 0 -> Torque, 2 -> RPM
  telegram.u16CoilsNo = 2; // number of elements (coils or registers) to read
  telegram.au16reg = au16data; // pointer to a memory array in the Arduino
  #endif

  #if rampTest
  currentRampValue = startRampValue;
  #endif

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
  #endif 

  printChange(5); // Prints the variables of interest current values
  
}

void loop()
{
  #if eRobExp
  switch (currentState)   // Handling the state machine
  {
    case STATE_IDLE:
      Serial.println("\n***--- MENU INBIODROID ---***");
      Serial.print("Set Control Mode: "); Serial.println(OPTION_SET_CONTROL_MODE - '0');
      Serial.print("Turn On: "); Serial.println(OPTION_TURN_ON - '0');
      Serial.print("Turn Off: "); Serial.println(OPTION_TURN_OFF - '0');
      Serial.print("Set Control: "); Serial.println(OPTION_SET_CONTROL - '0');
      Serial.print("Set Torque 0: "); Serial.println(OPTION_SET_TORQUE_0 - '0');
      Serial.print("Home Position: "); Serial.println(OPTION_HOME_POSITION - '0');
      Serial.print("Motor Response: "); Serial.println(OPTION_MOTOR_RESPONSE - '0');
      Serial.print("Change Values: "); Serial.println(OPTION_CHANGE_VALUES - '0');
      Serial.print("Go to zero: "); Serial.println(OPTION_GO_ZERO - '0');
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
        previousMillisPrintingFrecuency = millis();

        #if rampTest
          startRamp();
        #endif

        while(digitalRead(CTRL_PIN) && !flag)
        {
          #if buttonTest
          updateKd();
          #endif

          currentMillis = millis();
          if (currentMillis - previousMillisDuration >= (duration * 1000))
          {
            flag = true;
          }
          if (currentMillis - previousMillisFrecuency >= (1000.0/frecuency))
          {
            previousMillisFrecuency = currentMillis;
            control();
          }
          if (currentMillis - previousMillisPrintingFrecuency >= (printingFrecuency))
          {
            previousMillisPrintingFrecuency = currentMillis;
            printData();
          }
        }
        flag = false;

        #if buttonTest
          currentStateKd = false;
        #endif

        for (int i = 0; i < 10; i++)    //After each control signal, torque 0 is commanded
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
        getData();
        previousMillisPrintingFrecuency = millis();
        setpointTorque = 0.0;
        for (int i = 0; i < 100; i++)
        {
          getData();
          if (!erob_1.setTorque(setpointTorque, 2000))
          {
            Serial.println("ERROR: SET TORQUE 0 FAILED");
          }
          if (currentMillis - previousMillisPrintingFrecuency >= (printingFrecuency))
          {
            previousMillisPrintingFrecuency = currentMillis;
            printData();
          }
        }        
      }

      else if (input == OPTION_HOME_POSITION)
      {

        Serial.println("ERROR: SET HOME IS NOT AVAILABLE RIGHT NOW");
        /*
        if (!erob_1.setCurrentPositionAsZero())
        {
          Serial.println("ERROR: SET CURRENT POSITION AS ZERO FAILED");
        }
        else
        {
          Serial.println("SET CURRENT POSITION AS ZERO SUCCEEDED");
        }
        */
      }

      else if (input == OPTION_MOTOR_RESPONSE)
      {
        if (!erob_1.readFrame())
          {
            Serial.println("ERROR: READ MOTOR FAILED");
          }
      }

      else if (input == OPTION_CHANGE_VALUES)
      {     
        bool inputReceived = false;
        Serial.println("Press the correct character followed by the new value.");
        Serial.println("| S for Setpoint | P for kP | D for kD | T for Time | E for exit |");
        Serial.println("- For example: 'S5' sets the setpointTorque variable to 5");
        Serial.printf("- Remember the torque limits [%f  %f]\n", minTorque, maxTorque);        
        Serial.println("- The letter E will only print the current values without changing any of them");
        printChange(5); // Prints the variables of interest current values
        Serial.println("\nWaiting for command...");

        while (!inputReceived) {
          if (Serial.available() > 0) {
            cmd = Serial.read();
            if (cmd > 31) {
              if (cmd > 'Z') cmd -= 32;  // Convert to uppercase if necessary
              byte flags = 0;
              switch (cmd) {
                case 'S':
                  if (Serial.available() > 0) {
                    float tempTorque = Serial.parseFloat();
                    if (tempTorque < minTorque) {
                      setpointTorque = minTorque;
                      Serial.printf("Torque too low. Setting to minimum: %f\n", minTorque);
                    } else if (tempTorque > maxTorque) {
                      setpointTorque = maxTorque;
                      Serial.printf("Torque too high. Setting to maximum: %f\n", maxTorque);
                    } else {
                      setpointTorque = tempTorque;
                    }
                    flags = 1;
                  }
                  break;
                case 'P':
                  if (Serial.available() > 0) {
                    Kp = Serial.parseFloat();
                    flags = 2;
                  }
                  break;
                case 'D':
                  if (Serial.available() > 0) {
                    Kd = Serial.parseFloat();
                    flags = 3;
                  }
                  break;
                case 'T':
                  if (Serial.available() > 0) {
                    duration = Serial.parseFloat();
                    flags = 4;
                  }
                  break;                
                case 'E':
                  flags = 5;
                  break;
                default:
                  Serial.println("Invalid command. Please enter 'S', 'P', 'D', 'T', or 'E'.");
                  continue;
              }
              printChange(flags);
              inputReceived = true;  // Exit the loop after processing input
            }
          }
        }      
      }

      else if (input == OPTION_GO_ZERO)
      {
        Serial.println("Regresando a Cero");
        getData();
        previousMillisDuration = millis();
        previousMillisFrecuency = millis();
        previousMillisPrintingFrecuency = millis();

        while(digitalRead(CTRL_PIN) && !flag)
        {
          currentMillis = millis();
          if (currentMillis - previousMillisDuration >= (duration * 2000))
          {
            flag = true;
          }
          if (currentMillis - previousMillisFrecuency >= (1000.0/frecuency))
          {
            previousMillisFrecuency = currentMillis;
            goToZero();
          }
          if (currentMillis - previousMillisPrintingFrecuency >= (printingFrecuency))
          {
            previousMillisPrintingFrecuency = currentMillis;
            printData();
          }
        }
        flag = false;
        setpointTorque = 0.0;
        for (int i = 0; i < 10; i++)    //After each control signal, torque 0 is commanded
        {
          if (!erob_1.setTorque(setpointTorque, 2000))
          {
            Serial.println("ERROR: SET TORQUE 0 FAILED");
          }
          delay(100);
        }        
        Serial.println("Zero Reached!!");
      }

      else
      {
        Serial.println("ERROR: INVALID OPCION");
      }
      currentState = STATE_IDLE;
      break;
  }
  #endif 
}