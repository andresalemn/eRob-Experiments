#include "eRob.hpp"
#include "def.hpp"

#define MILLIS_LIMIT_UNTIL_RETRY 5


//Torque constant Kt(Nm/A) 
#define EROB_70_KT  7.92f
#define EROB_90_KT  8.40f
#define EROB_110_KT 8.04f

//Rated Torque (A) ToDo: Check Values
#define EROB_70_RC  0.20f
// #define EROB_90_RC  18.00f
#define EROB_90_RC  6.70f
#define EROB_110_RC 0.20f 

const eRob::MotorType eRob::eRob_70{EROB_70_KT, EROB_70_RC};
const eRob::MotorType eRob::eRob_90{EROB_90_KT, EROB_90_RC};
const eRob::MotorType eRob::eRob_110{EROB_110_KT, EROB_110_RC};                  

eRob::eRob(const MotorType & motor_type, const uint8_t CANID, const uint8_t chipSelect, const uint8_t intPin, SPIClass & spi, const bool doBegin)
    : m_CANID(CANID), m_interruptPin(intPin),  m_mcp2515{chipSelect, spi, doBegin}, m_motor_type(motor_type), m_addressNVS(CANID * sizeof(int32_t))
{
}

eRob::~eRob()
{
}

bool eRob::m_sendBlocking(const can_frame & canMsg , unsigned long timeout_us)
{
     MCP2515::ERROR response_code;
    unsigned long t_ini = micros();
    while ( (response_code = m_mcp2515.sendMessage(&canMsg)) != MCP2515::ERROR_OK and (micros()-t_ini) < timeout_us)
    {   
        #if DEBUG_LOG
            Serial.println("Waiting free buffer to send command ");
        #endif
    }
    if ( response_code != MCP2515::ERROR_OK){
        #if DEBUG_LOG
            Serial.print("Failed sending message");
        #endif
        return false;
    }
    return true;
}

bool eRob::initialize(const CAN_SPEED can_speed, CAN_CLOCK can_clock)
{
    #if DEBUG_LOG
        //Serial.print("Initializing motor: "); Serial.println(m_name);
    #endif
    if ( m_mcp2515.reset() != MCP2515::ERROR_OK)
    {
        #if DEBUG_LOG
            Serial.println("!!!Error Resetting MCP2515.");
        #endif
        return false;
    }
    if ( m_mcp2515.setBitrate(can_speed, can_clock) != MCP2515::ERROR_OK)
    {
        #if DEBUG_LOG
            Serial.println("!!!Error Setting MCP2515 Bitrate.");
        #endif
        return false;
    }
    if ( m_mcp2515.setNormalMode() != MCP2515::ERROR_OK)
    {
        #if DEBUG_LOG
            Serial.println("!!!Error Setting MCP2515 normal mode.");
        #endif
        return false;
    }
    return true;
}

bool eRob::nmtMessage(uint8_t cmd, unsigned long timeout_us)
{
    can_frame canMsg;
    canMsg.can_id  = COB_ID_NMT;
    canMsg.can_dlc = 0x02;
    canMsg.data[0] = static_cast<uint8_t>(cmd);
    canMsg.data[1] = static_cast<uint8_t>(m_CANID);
    return m_sendBlocking(canMsg, timeout_us); 
}

bool eRob::syncMessage(unsigned long timeout_us)
{
    can_frame canMsg;
    canMsg.can_id  = 0x80;
    canMsg.can_dlc = 0x00;
    return m_sendBlocking(canMsg, timeout_us);  
}

//ToDo: Verify that heartbeat messages can be sent
bool eRob::heartbeatMessage()
{
    can_frame canMsgTx;
    canMsgTx.can_id  = COB_ID_HEARTBEAT + m_CANID;
    canMsgTx.can_dlc = 0x00;
    bool result = (m_mcp2515.sendMessage(&canMsgTx) == MCP2515::ERROR_OK); 
    return result; 
}


template<typename DataType>
bool eRob::wSdoMessage(uint16_t idx, uint8_t sub_idx, DataType data, unsigned long timeout_us)
{
    can_frame canMsg;
    constexpr size_t data_size = sizeof(DataType);
    canMsg.can_id  = COB_ID_SDO_CLIENT + m_CANID;
    canMsg.can_dlc = 0x08;
    canMsg.data[0] = BASE_COMMAND_CODE - (VALUE_EACH_BYTE * data_size);
    canMsg.data[1] = idx;
    canMsg.data[2] = idx >> 8;
    canMsg.data[3] = sub_idx;
    canMsg.data[4] = static_cast<uint8_t>(data);
    canMsg.data[5] = static_cast<uint8_t>(data >> 8);
    canMsg.data[6] = static_cast<uint8_t>(data >> 16);
    canMsg.data[7] = static_cast<uint8_t>(data >> 24);
    if(!m_sendBlocking(canMsg, timeout_us))
    {
        return false;
    }
    return readMotorResponse(timeout_us);
}

bool eRob::rSdoMessage(uint16_t idx, uint8_t sub_idx, unsigned long timeout_us)
{
    can_frame canMsg;
    canMsg.can_id  = COB_ID_SDO_CLIENT + m_CANID;
    canMsg.can_dlc = 0x08;
    canMsg.data[0] = READ_N_BYTES;
    canMsg.data[1] = idx;
    canMsg.data[2] = idx >> 8;
    canMsg.data[3] = sub_idx;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    if(!m_sendBlocking(canMsg, timeout_us))
    {
        return false;
    }
    return readMotorResponse(timeout_us);
}

bool eRob:: m_setOperationState(uint16_t state, unsigned long timeout_us)
{
    can_frame canMsg;
    canMsg.can_id  = COB_ID_TX_PDO_1 + m_CANID;
    canMsg.can_dlc = 0x06;
    canMsg.data[0] = state;
    canMsg.data[1] = state << 8;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    return m_sendBlocking(canMsg, timeout_us);
}

bool eRob::readMotorResponse(unsigned long timeout_us)
{
    unsigned long t_ini = micros();
    bool was_response_received;
    while ( !(was_response_received = m_readMotorResponse()) and (micros()-t_ini) < timeout_us)
    {
        #if DEBUG_LOG
            //Serial.println("Waiting Response");
        #endif
    }
    if ( !was_response_received)
    {
        #if DEBUG_LOG
            Serial.print(": NO ANSWER \n");
        #endif
        return false;
    }
    else 
    { 
        #if DEBUG_LOG
            //Serial.print("Success time: "); Serial.println(i+1);
        #endif
    }
    return true;
}

bool eRob::readMotorResponse()
{
    return m_readMotorResponse();
}

bool eRob::m_readMotorResponse()
{
    can_frame canMsg;
    if(m_mcp2515.readMessage(&canMsg) != MCP2515::ERROR::ERROR_OK)
    { 
        return false;
    }
    switch (canMsg.can_id - m_CANID)
    {
        case (COB_ID_EMERGENCY):
        {
        
            #if DEBUG_LOG
                uint16_t errorCode = (canMsg.data[0] << 8) + canMsg.data[0];
                Serial.print("Error code: "); Serial.println(errorCode , HEX);
                Serial.print("Error Register: "); Serial.println(canMsg.data[2]);
            #endif
            return false;
            break;
        }
        case COB_ID_HEARTBEAT:
        {
            #if DEBUG_LOG
                Serial.print("State: "); Serial.println(canMsg.data[0]);
            #endif
            return true;
            break;
        }
        case COB_ID_SDO_SERVER:
        {
            switch (canMsg.data[0])
            {
                case WRITE_N_BYTES_RESPONSE:
                {
                    #if DEBUG_LOG
                        Serial.println("Successfully Write SDO Message.");
                    #endif
                    return true;
                    break;
                }
                case READ_ONE_BYTES_RESPONSE:
                {
                    m_dataResponse = canMsg.data[4];
                    #if DEBUG_LOG
                        Serial.println("Successfully Read SDO Message.");
                        Serial.print("Data: "); Serial.println(m_dataResponse, HEX);
                    #endif
                    return true;
                    break;
                }
                case READ_TWO_BYTES_RESPONSE:
                {
                    m_dataResponse = (canMsg.data[5] << 8) + canMsg.data[4];
                    #if DEBUG_LOG
                        Serial.println("Successfully Read SDO Message.");
                        Serial.print("Data: "); Serial.println(m_dataResponse, HEX);
                    #endif
                    return true;
                    break;
                }
                case READ_THREE_BYTES_RESPONSE:
                {
                    m_dataResponse = (canMsg.data[6] << 16) +(canMsg.data[5] << 8) + canMsg.data[4];
                    #if DEBUG_LOG
                        Serial.println("Successfully Read SDO Message.");
                        Serial.print("Data: "); Serial.println(m_dataResponse, HEX);
                    #endif
                    return true;
                    break;
                }
                case READ_FOUR_BYTES_RESPONSE:
                {
                    m_dataResponse = (canMsg.data[7] << 24) + (canMsg.data[6] << 16) +(canMsg.data[5] << 8) + canMsg.data[4];
                    #if DEBUG_LOG
                        Serial.println("Successfully Read SDO Message.");
                        Serial.print("Data: "); Serial.println(m_dataResponse, HEX);
                    #endif
                    return true;
                    break;
                }
                case EXCEPTION_RESPONSE:
                {
                    #if DEBUG_LOG
                        m_dataResponse = (canMsg.data[7] << 24) + (canMsg.data[6] << 16) +(canMsg.data[5] << 8) + canMsg.data[4];
                        Serial.println("Failed to Write/Read SDO Message.");
                        Serial.print("Abort Code: "); Serial.println(m_dataResponse, HEX);
                    #endif
                    return false;
                    break;
                }
                default:
                {
                    #if DEBUG_LOG
                        Serial.println("Command code was not recognized.");
                    #endif
                    return false;
                    break;
                }
            }
        }
        case COB_ID_RX_PDO_1:
        {
            //Torque without sensor
            // m_torqueRAW = (canMsg.data[3] << 8) + canMsg.data[2];
            // m_torque = m_motor_type.KT * ((static_cast<float>(m_torqueRAW) * m_motor_type.RC) / 1000.0);
            //Torque with sensor
            int32_t sensorTorque =  (canMsg.data[5] << 24) + (canMsg.data[4] << 16) + (canMsg.data[3] << 8) + canMsg.data[2];
            m_torque = sensorTorque / 1000.0f;
            
            #if DEBUG_LOG
                Serial.print("Torque Sensor : "); Serial.println(m_torque);
                // Serial.print("Torque RAW: "); Serial.println(m_torqueRAW);
            #endif
            return true;
            break;
        }
        case COB_ID_RX_PDO_2:
        {   
            m_positionRAW = (canMsg.data[3] << 24) + (canMsg.data[2] << 16) + (canMsg.data[1] << 8) + canMsg.data[0];
            m_position = (static_cast<float>(m_positionRAW - m_positionOffset) / ENCODER_RESOLUTION) * TWO_PI; //Check eRob Rotary Actuator User Manual V3.37.pdf pag. 41
            m_velocityRAW = (canMsg.data[7] << 24) + (canMsg.data[6] << 16) + (canMsg.data[5] << 8) + canMsg.data[4];
            m_velocity = (static_cast<float>(m_velocityRAW) / ENCODER_RESOLUTION) * TWO_PI; //Check eRob Rotary Actuator User Manual V3.37.pdf pag. 53
            #if DEBUG_LOG
                Serial.print("Position : "); Serial.println(m_position);
                Serial.print("PositionRAW : "); Serial.println(m_positionRAW);
                Serial.print("PositionOffset : "); Serial.println(m_positionOffset);
                Serial.print("Velocity : "); Serial.println(m_velocity);
            #endif
            return true;
            break;
        }
        default:
        {
            #if DEBUG_LOG
                Serial.println("COB-ID was not recognized.");
            #endif
            return false;
            break;
        }
    }
    return true;   
}


bool eRob::m_stopRemoteNode()
{
    return nmtMessage(NMT_STOP_NODE); 
}


bool eRob::m_resetCommunication()
{
    if (!nmtMessage(NMT_RESET_COMUNICATION))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;   
    }
    if(!readMotorResponse(5000))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT RECEIVED");
        #endif
    }
    return true;
}


bool eRob::m_setOperationMode()
{
    // uint8_t data = 0x01; //Position Mode
    // uint8_t data = 0x03; //Velocity Mode
    uint8_t data = 0x04; //Torque Mode
    if (!wSdoMessage(OPERATION_MODES, INDEX_0, data))
    {
        return false;   
    }
    if(!rSdoMessage(CHECK_OPERATION_MODE, INDEX_0))
    {
        return false; 
    }
    return true;
}


bool eRob::m_disableCOBIDSync()
{
    uint32_t data = 0x80; //ToDo: WTF with this >:[, make a well function. 
    // uint32_t data = 0x40000080;
    if (!wSdoMessage(COB_ID_SYNC_MESSAGE, INDEX_0, data))
    {
        return false;
    }
    return true;
}


bool eRob::m_setCommunication_Cycle(uint32_t data)
{
    //us
    if(!wSdoMessage(COMMUNICATION_CYCLE_PERIOD, INDEX_0, data))
    {
        return false;
    }
    return true;
}


bool eRob::m_tPDOMapping()
{
    uint32_t dataFourBytes = DISABLE_TPDO_1 + m_CANID;
    if(!wSdoMessage(TPDO_COMMUNICATION_PARAMETER_1, INDEX_COB_ID_PDO, dataFourBytes))
    {
        return false;
    }

    // uint8_t dataOneBytes = 0xFF;//SYNC_BEHAVIOR;
    uint8_t dataOneBytes = SYNC_BEHAVIOR;
    if(!wSdoMessage(TPDO_COMMUNICATION_PARAMETER_1, INDEX_TRANSMISSION_TYPE, dataOneBytes))
    {
        return false;
    }

    dataOneBytes = 0x00;
    if(!wSdoMessage(TPDO_MAPPING_PARAMETER_1, INDEX_NUMBER_PDO_MAPS, dataOneBytes))
    {
        return false;
    }

    uint8_t objectDataLength = 0x10; //bits
    uint8_t objectSubIndex = INDEX_0; 
    uint16_t objectIndex = STATUS_WORD;  
    uint32_t PDOParameter =(objectIndex << 16) + (objectSubIndex << 8) + objectDataLength;
    if(!wSdoMessage(TPDO_MAPPING_PARAMETER_1, INDEX_PDO_MAPPING_ENTRY_1, PDOParameter))
    {
        return false;
    }

    //torque without sensor
    // objectDataLength = 0x10; //bits
    // objectSubIndex = INDEX_0; 
    // objectIndex = TORQUE_ACTUAL_VALUE; 

    // torque with sensor
    objectDataLength = 0x20; //bits
    objectSubIndex = INDEX_0; 
    objectIndex = TORQUE_SENSOR; 

    PDOParameter = (objectIndex << 16) + (objectSubIndex << 8) + objectDataLength;
    if(!wSdoMessage(TPDO_MAPPING_PARAMETER_1, INDEX_PDO_MAPPING_ENTRY_2, PDOParameter))
    {
        return false;
    }

    dataOneBytes = 0x02;
    if(!wSdoMessage(TPDO_MAPPING_PARAMETER_1, INDEX_NUMBER_PDO_MAPS, dataOneBytes))
    {
        return false;
    }

    dataFourBytes = ENABLE_TPDO_1 + m_CANID;
    if(!wSdoMessage(TPDO_COMMUNICATION_PARAMETER_1, INDEX_COB_ID_PDO, dataFourBytes))
    {
        return false;
    }

    dataFourBytes = DISABLE_TPDO_2 + m_CANID;
    if(!wSdoMessage(TPDO_COMMUNICATION_PARAMETER_2, INDEX_COB_ID_PDO, dataFourBytes))
    {
        return false;
    }

    dataOneBytes = SYNC_BEHAVIOR;
    if(!wSdoMessage(TPDO_COMMUNICATION_PARAMETER_2, INDEX_TRANSMISSION_TYPE, dataOneBytes))
    {
        return false;
    }

    dataOneBytes = 0x00;
    if(!wSdoMessage(TPDO_MAPPING_PARAMETER_2, INDEX_NUMBER_PDO_MAPS, dataOneBytes))
    {
        return false;
    }

    objectDataLength = 0x20; //bits
    objectSubIndex = INDEX_0; 
    objectIndex = POSITION_ACTUAL_VALUE;  
    PDOParameter = (objectIndex << 16) + (objectSubIndex << 8) + objectDataLength;
    if(!wSdoMessage(TPDO_MAPPING_PARAMETER_2, INDEX_PDO_MAPPING_ENTRY_1, PDOParameter))
    {
        return false;
    }

    objectDataLength = 0x20; //bits
    objectSubIndex = INDEX_0; 
    objectIndex = VELOCITY_ACTUAL_VALUE;  
    PDOParameter = (objectIndex << 16) + (objectSubIndex << 8) + objectDataLength;
    if(!wSdoMessage(TPDO_MAPPING_PARAMETER_2, INDEX_PDO_MAPPING_ENTRY_2, PDOParameter))
    {
        return false;
    }
    dataOneBytes = 0x02;
    if(!wSdoMessage(TPDO_MAPPING_PARAMETER_2, INDEX_NUMBER_PDO_MAPS, dataOneBytes))
    {
        return false;
    }

    dataFourBytes = ENABLE_TPDO_2 + m_CANID;
    if(!wSdoMessage(TPDO_COMMUNICATION_PARAMETER_2, INDEX_COB_ID_PDO, dataFourBytes))
    {
        return false;
    }
    return true;
}

bool eRob::m_rPDOMapping()
{
    uint32_t dataFourBytes = DISABLE_RPDO_1 + m_CANID;
    if(!wSdoMessage(RPDO_COMMUNICATION_PARAMETER_1, INDEX_COB_ID_PDO, dataFourBytes))
    {
        return false;
    }

    uint8_t dataOneBytes = SYNC_BEHAVIOR;
    if(!wSdoMessage(RPDO_COMMUNICATION_PARAMETER_1, INDEX_TRANSMISSION_TYPE, dataOneBytes))
    {
        return false;
    }

    dataOneBytes = 0x00;
    if(!wSdoMessage(RPDO_MAPPING_PARAMETER_1, INDEX_NUMBER_PDO_MAPS, dataOneBytes))
    {
        return false;
    }

    uint8_t objectDataLength = 0x10; //bits
    uint8_t objectSubIndex = INDEX_0; 
    uint16_t objectIndex = CONTROL_WORD;  
    uint32_t PDOParameter = (objectIndex << 16) + (objectSubIndex << 8) + objectDataLength;
    if(!wSdoMessage(RPDO_MAPPING_PARAMETER_1, INDEX_PDO_MAPPING_ENTRY_1, PDOParameter))
    {
        return false;
    }

    objectDataLength = 0x10; //bits
    objectSubIndex = INDEX_0; 
    objectIndex = TARGET_TORQUE;  
    PDOParameter = (objectIndex << 16) + (objectSubIndex << 8) + objectDataLength;
    if(!wSdoMessage(RPDO_MAPPING_PARAMETER_1, INDEX_PDO_MAPPING_ENTRY_2, PDOParameter))
    {
        return false;
    }

    dataOneBytes = 0x02;
    if(!wSdoMessage(RPDO_MAPPING_PARAMETER_1, INDEX_NUMBER_PDO_MAPS, dataOneBytes))
    {
        return false;
    }

    dataFourBytes = ENABLE_RPDO_1 + m_CANID;
    if(!wSdoMessage(RPDO_COMMUNICATION_PARAMETER_1, INDEX_COB_ID_PDO, dataFourBytes))
    {
        return false;
    }
    return true;
}

bool eRob::m_startRemoteNode()
{
    if(!nmtMessage(NMT_START_NODE))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    }
    if(!heartbeatMessage())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    } 
    if(!syncMessage())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    } 
    if (!readMotorResponse(2000))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT RECEIVED");
        #endif
    }
    if (!readMotorResponse(2000))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT RECEIVED");
        #endif
    }
    return true;
}

bool eRob::m_enable()
{
    uint16_t state = FAULT_RESET;
    if(!m_setOperationState(state))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    }
    state = ENABLE_VOLTAGE_QUICK_STOP;
    if(!m_setOperationState(state))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    }
    state = SWITCH_ON;
    if(!m_setOperationState(state))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    }
    state = OPERATION_ENABLE;
    if(!m_setOperationState(state))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    }
    if(!syncMessage())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    } 
    if (!readMotorResponse(2000))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT RECEIVED");
        #endif
    }
    if (!readMotorResponse(2000))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT RECEIVED");
        #endif
    }
    return true;
}


bool eRob::m_disable()
{
    uint16_t state = OPERATION_DISABLE;
    if(!m_setOperationState(state))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    }
    return true;
}


bool eRob::setcontrolMode()
{
    if (!readMotorResponse())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT RECEIVED");
        #endif
    }
    
    if(!m_stopRemoteNode())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: STOP REMOTE NODE FAILED");
        #endif
        return false;
    }
    if(!m_resetCommunication())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: RESET COMMUNICATION FAILED");
        #endif
        return false;
    }
    if(!m_setOperationMode())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: SET OPERATION MODE FAILED");
        #endif
        return false;
    }
    if(!m_disableCOBIDSync())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: DISABLE COB ID SYNC FAILED");
        #endif
        return false;
    }
    if(!m_setCommunication_Cycle())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: SET COMMUNICATION CYCLE FAILED");
        #endif
        return false;
    }
    // Get position offset from NVS
    EEPROM.get( m_addressNVS, m_positionOffset);

    if(!m_tPDOMapping())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: TPDO MAPPING FAILED");
        #endif
        return false;
    }
    if(!m_rPDOMapping())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: RPPDO MAPPING FAILED");
        #endif
        return false;
    }
    if(!m_startRemoteNode())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: START REMOTE NODE FAILED");
        #endif
        return false;
    }

    return true;
}

bool eRob::turnOn()
{
    if(!m_enable())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: ENABLE FAILED");
        #endif
        return false;
    }
    return true;
}

bool eRob::turnOff()
{
    if(!m_disable())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: DISABLE FAILED");
        #endif
        return false;
    }
    return true;
}

bool eRob::readFrame()
{
  can_frame canMsgRx;
  MCP2515::ERROR response_code = m_mcp2515.readMessage(&canMsgRx);
  if(response_code != MCP2515::ERROR::ERROR_OK)
  {
      return false;
  }
  Serial.print("COB-ID : "); Serial.println(canMsgRx.can_id, HEX);
  Serial.print("PAYLOAD : "); Serial.println(canMsgRx.can_dlc, HEX);

  for(int i = 0; i <= (canMsgRx.can_dlc - 1); i++)
  {
    Serial.print("DATA ["); Serial.print(i); Serial.print("]: "); Serial.println(canMsgRx.data[i], HEX);
  }
  Serial.println();
  return true;
}


//ToDo: Test Funcionality
bool eRob::setCurrentPositionAsZero()
{
    int32_t positionOffset = 0;
    if(!rSdoMessage(POSITION_ACTUAL_VALUE, INDEX_0))
    {
        return false; 
    }
    positionOffset = static_cast<int32_t>(m_dataResponse);
    m_positionOffset = positionOffset; 
    EEPROM.put(m_addressNVS, m_positionOffset);
    return true;
}

bool eRob::readRatedCurrent()
{
    if(!rSdoMessage(RATED_CURRENT, INDEX_0))
    {
        return false; 
    }
    Serial.println(m_dataResponse);
    Serial.println(m_dataResponse, HEX);
    return true;
}

bool eRob::m_saveParameters()
{
    uint32_t save = 0x65766173; //"save"
    if (!wSdoMessage(SAVE_PARAMETERS, INDEX_1, save))
    {
        return false;   
    }
    return true;
}


bool eRob::m_sendTorque(float torqueSetpoint)
{
    int16_t perThousandRatedCurrent = static_cast<int16_t>(torqueSetpoint * 1000.0 / (m_motor_type.KT * m_motor_type.RC));
    can_frame canMsg;
    canMsg.can_id  = COB_ID_TX_PDO_1 + m_CANID;
    canMsg.can_dlc = 0x06;
    canMsg.data[0] = 0x2F;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = perThousandRatedCurrent;
    canMsg.data[3] = perThousandRatedCurrent >> 8;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    bool result = (m_mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK); 
    return result;
}

bool eRob::setTorque(float torqueSetpoint)
{
    //Normal mode (Polling based)
    if(!m_is_auto_mode_running) return m_sendTorque(torqueSetpoint);

    //Auto mode (Interrupt driven)
    if (m_is_response_ready)
    {
        m_is_response_ready = false;
        uint8_t irq = m_mcp2515.getInterrupts();
        if (irq & MCP2515::CANINTF_MERRF)
        {
            Serial.print("\n\n!!!!!ERROR MERF (ERROR IN MESSAGE TRANSMISSION OR RECEPTION)!!!"); Serial.print("m_name"); Serial.print("\n\n");
            m_mcp2515.clearMERR();
            m_mcp2515.clearInterrupts();
            return false; //ToDo: Is it Correct?
        }
        if (irq & MCP2515::CANINTF_ERRIF)
        {
            Serial.print("\n\n!!!!!!!ERROR BUFFER FULL!!!!!!"); Serial.print("m_name"); Serial.print("\n\n");
            m_mcp2515.clearRXnOVRFlags();
            m_mcp2515.clearERRIF();
            m_mcp2515.clearInterrupts();
            return false; //ToDo: Is it Correct?
        }
        return m_sendTorque(torqueSetpoint);
    }

    else if ((millis() - m_last_response_time_ms) < MILLIS_LIMIT_UNTIL_RETRY) //In case setTorque() was called again before message reception.
    {
        return true;
    }

    else
    {
        //Serial.print("\t Millis: "); Serial.print(millis()); Serial.print("\tLast message"); Serial.print(m_last_response_time_ms); Serial.print("\tRetrying to recover "); Serial.println(m_name); 
        if ((millis() - m_last_retry_time_ms) > MILLIS_LIMIT_UNTIL_RETRY)
        {
            m_last_retry_time_ms = millis();
            m_emptyMCP2515buffer();
            m_sendTorque(torqueSetpoint);
        }
        return false;
    }
}

bool eRob::setTorque(float torqueSetpoint, unsigned long timeout_us)
{
    if (m_is_auto_mode_running) 
    {
        return setTorque(torqueSetpoint);
    }
    bool messageStatus;
    unsigned long t_ini = micros();
    while( !(messageStatus = setTorque(torqueSetpoint)) and (micros()-t_ini) < timeout_us)
    {
        // #if DEBUG_LOG
        Serial.println(messageStatus);
        Serial.println("Send Retry!");      
        // #endif     
    }
    if(!messageStatus)
    {
        #if DEBUG_LOG
            Serial.print("Failed sending message");
        #endif
        return false;
    }
    if(!syncMessage())
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT SENT");
        #endif
        return false;
    } 
    if (!readMotorResponse(timeout_us))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT RECEIVED");
        #endif
    }
    if (!readMotorResponse(timeout_us))
    {
        #if DEBUG_LOG
            Serial.println("ERROR: FRAME WAS NOT RECEIVED");
        #endif
    }
    return true;
}

void eRob::m_emptyMCP2515buffer()
{
    can_frame devnull;
    while(m_mcp2515.readMessage(&devnull) == MCP2515::ERROR_OK){ /*Serial.println("Vaciando buffer...");*/ }
    return;
}

void eRob::startAutoMode(void (*ISR_callback)(void))
{
    //Serial.print(m_name); Serial.println("Started auto mode"); 
    m_is_auto_mode_running = true;
    m_is_response_ready = true;
    m_emptyMCP2515buffer();
    m_mcp2515.clearInterrupts();
    pinMode(m_interruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(m_interruptPin), ISR_callback, FALLING);
    m_last_response_time_ms = millis();
    return;
}

void eRob::stopAutoMode()
{
    m_is_auto_mode_running = false;
    detachInterrupt(digitalPinToInterrupt(m_interruptPin));
    m_emptyMCP2515buffer();
    m_mcp2515.clearInterrupts();
    return;
}

void eRob::handleInterrupt(void)
{
    m_last_response_time_ms = millis();
    m_is_response_ready = true;
}