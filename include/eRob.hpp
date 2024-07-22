#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <EEPROM.h>

#define ENCODER_RESOLUTION 524288
#define TIMEOUT_US 2000
// #define INT32_T_LENGHT 4

class eRob
{
public:
    struct MotorType
    {
        const float KT;
        const float RC;
        constexpr MotorType(float _kt, float _rc) : KT(_kt), RC(_rc){}  //Constructor
    };
    static const MotorType eRob_70;
    static const MotorType eRob_90;
    static const MotorType eRob_110;

    eRob(const MotorType & motor_type, const uint8_t CANID, const uint8_t chipSelect, const uint8_t intPin, SPIClass & spi = SPI, const bool doBegin = false);
    ~eRob();
    bool initialize(const CAN_SPEED can_speed = CAN_1000KBPS, CAN_CLOCK can_clock = MCP_16MHZ);
    bool nmtMessage(uint8_t cmd, unsigned long timeout_us = TIMEOUT_US);
    bool syncMessage(unsigned long timeout_us = TIMEOUT_US);
    bool heartbeatMessage();
    template<typename DataType>
    bool wSdoMessage(uint16_t idx, uint8_t sub_idx, DataType data, unsigned long timeout_us = TIMEOUT_US);
    bool rSdoMessage(uint16_t idx, uint8_t sub_idx, unsigned long timeout_us = TIMEOUT_US);

    bool setcontrolMode();
    bool turnOn();
    bool turnOff();
    bool readFrame();
    bool readMotorResponse(unsigned long timeout_us);
    bool readMotorResponse();
    bool setCurrentPositionAsZero();
    bool readRatedCurrent();

    //Public "getters" to motor response variables. 
    float getPosition() const {return m_position;}
    const float getVelocity() const {return m_velocity;}
    const float getTorque() const {return m_torque;}

    bool setTorque(float torqueSetpoint, unsigned long timeout_us);
    bool setTorque(float torqueSetpoint);
    void startAutoMode(void (*ISR_callback)(void));
    void stopAutoMode();
    void handleInterrupt(void); 

private:
    const uint8_t m_CANID;
    const uint8_t m_interruptPin;
    MCP2515 m_mcp2515;
    const MotorType m_motor_type;

    float m_position;
    int32_t m_positionRAW;
    float m_velocity;
    int32_t m_velocityRAW;
    float m_torque;
    int16_t m_torqueRAW;

    uint32_t m_dataResponse;
    int32_t m_positionOffset;
    const int m_addressNVS;

    bool m_is_auto_mode_running = false;
    bool m_is_response_ready;
    unsigned long m_last_response_time_ms;
    unsigned long m_last_retry_time_ms;

    bool motorOn = false;
    
    
    
    bool m_sendBlocking(const can_frame & can_msg , unsigned long timeout_us = TIMEOUT_US);
    // bool m_receiveBlocking(const can_frame & can_msg , unsigned long timeout_us);
    bool m_setOperationState(uint16_t control, unsigned long timeout_us = TIMEOUT_US);
    
    bool m_readMotorResponse();
    bool m_stopRemoteNode();
    bool m_resetCommunication();
    bool m_setOperationMode();
    bool m_disableCOBIDSync();
    bool m_setCommunication_Cycle(uint32_t data = 1000);
    bool m_tPDOMapping(); //Motor sends values
    bool m_rPDOMapping(); //Motor reads values
    bool m_startRemoteNode();
    bool m_enable();
    bool m_disable();
    bool m_saveParameters();
    bool m_sendTorque(float torqueSetpoint);
    void m_emptyMCP2515buffer();


    
};


