// mintasca_sdk.hpp
#ifndef MINTASCA_SDK_HPP
#define MINTASCA_SDK_HPP

#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <map>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

// 执行器信息结构
struct ActuatorInfo {
    uint8_t id;
    std::string serial_number;
    float current_position;
    uint8_t mode;
    
    ActuatorInfo() : id(0), current_position(0.0f), mode(0) {}
    ActuatorInfo(uint8_t _id, const std::string& _serial, float _position, uint8_t _mode)
        : id(_id), serial_number(_serial), current_position(_position), mode(_mode) {}
};

class MintascaSDK {
public:
    // 默认配置常量
    static const std::string DEFAULT_IP_ADDRESS;
    static const uint16_t DEFAULT_PORT;
    
    // Constructor and Destructor
    MintascaSDK();
    ~MintascaSDK();

    // Initialization and connection methods
    bool initialize(); // 使用默认IP和端口初始化
    bool initialize(const std::string& ip_address, uint16_t port = 2000);
    bool connect();
    void disconnect();
    
    // Handshake and device discovery
    bool handshake();
    bool discoverActuators();
    std::vector<uint8_t> queryActuatorAddresses();
    
    // Debug method for testing
    std::vector<uint8_t> testQueryActuatorAddresses();
    
    // Debug mode
    void setDebugMode(bool debug) { debug_mode_ = debug; }
    bool isDebugMode() const { return debug_mode_; }
    
    // Accessor for actuator information
    const std::map<uint8_t, ActuatorInfo>& getActuatorInfo() const { return actuators_; }

    // Read methods
    float readCurrentSpeed(uint8_t address);
    float readCurrentPosition(uint8_t address);
    float readCurrentTorque(uint8_t address);
    float readCurrent(uint8_t address);  // 读取当前电流值
    uint32_t readErrorCode(uint8_t address);
    uint8_t readModeStatus(uint8_t address);
    float readTargetPosition(uint8_t address);
    float readTargetSpeed(uint8_t address);
    float readTargetTorque(uint8_t address);
    float readPositionDeviation(uint8_t address);
    float readSpeedDeviation(uint8_t address);
    float readTorqueDeviation(uint8_t address);
    uint32_t readCurrentPositionPulse(uint8_t address);
    uint32_t readTargetPositionPulse(uint8_t address);
    uint32_t readPositionDeviationPulse(uint8_t address);
    float readRatedCurrent(uint8_t address);
    float readRatedSpeed(uint8_t address);
    float readRatedTorque(uint8_t address);
    uint8_t readActuatorID(uint8_t address);
    uint32_t readCommunicationBaudRate(uint8_t address);
    uint8_t readDriverTemperature(uint8_t address);        // 修改返回类型为uint8_t
    uint8_t readActuatorTemperature(uint8_t address);      // 修改返回类型为uint8_t
    float readBusVoltage(uint8_t address);
    float readPhaseACurrent(uint8_t address);
    float readPhaseBCurrent(uint8_t address);
    float readPhaseCCurrent(uint8_t address);
    uint32_t readEncoderPosition(uint8_t address);
    uint32_t readEncoderSpeed(uint8_t address);
    float readCurrentKp(uint8_t address);
    float readCurrentKi(uint8_t address);
    float readCurrentKd(uint8_t address);
    float readSpeedKp(uint8_t address);
    float readSpeedKi(uint8_t address);
    float readSpeedKd(uint8_t address);
    float readPositionKp(uint8_t address);
    float readPositionKi(uint8_t address);
    float readPositionKd(uint8_t address);
    
    // Write methods
    bool writeTargetPosition(uint8_t actuator_id, float position);
    bool writeTargetSpeed(uint8_t actuator_id, float speed);
    bool writeTargetTorque(uint8_t actuator_id, float torque);
    bool writeActuatorID(uint8_t actuator_id, uint8_t new_id);
    bool writeCommunicationBaudRate(uint8_t actuator_id, uint32_t baud_rate);
    bool setCurrentKp(uint8_t actuator_id, float kp);
    bool setCurrentKi(uint8_t actuator_id, float ki);
    bool setCurrentKd(uint8_t actuator_id, float kd);
    bool setSpeedKp(uint8_t actuator_id, float kp);
    bool setSpeedKi(uint8_t actuator_id, float ki);
    bool setSpeedKd(uint8_t actuator_id, float kd);
    bool setPositionKp(uint8_t actuator_id, float kp);
    bool setPositionKi(uint8_t actuator_id, float ki);
    bool setPositionKd(uint8_t actuator_id, float kd);
    bool clearError(uint8_t actuator_id);
    bool saveParameters(uint8_t actuator_id);
    bool restoreParameters(uint8_t actuator_id);
    bool controlSwitching(uint8_t actuator_id, uint8_t mode);
    bool motorShutdown(uint8_t actuator_id);
    bool faultShutdown(uint8_t actuator_id);
    bool enableActuator(uint8_t actuator_id);
    bool disableActuator(uint8_t actuator_id);

private:
    // Internal helper methods
    std::vector<uint8_t> sendCommand(uint8_t address, uint8_t command, 
                                     const std::vector<uint8_t>& data = {});
    uint16_t calculateCRC(const std::vector<uint8_t>& data);
    std::vector<uint8_t> floatToIQ24(float value);
    float IQ24ToFloat(const std::vector<uint8_t>& data);
    std::vector<uint8_t> uint32ToBytes(uint32_t value);
    uint32_t bytesToUint32(const std::vector<uint8_t>& data);
    uint8_t bytesToUint8(const std::vector<uint8_t>& data);
    
    // Debug helper
    void debugPrint(const std::string& message, const std::vector<uint8_t>& data);
    
    // Connection parameters
    std::string ip_address_;
    uint16_t port_;
    int socket_fd_;
    struct sockaddr_in server_addr_;
    
    // Actuator information
    std::map<uint8_t, ActuatorInfo> actuators_;
    
    // Debug mode
    bool debug_mode_;
    
    // Constants
    static const uint8_t FRAME_HEADER;
    static const uint8_t FRAME_TAIL;
    static const uint8_t BROADCAST_ADDRESS;
    
    // Command codes (based on protocol documentation)
    static const uint8_t CMD_READ_CURRENT_SPEED;         // 0x01 当前速度值
    static const uint8_t CMD_READ_CURRENT_POSITION;      // 0x06 当前位置值
    static const uint8_t CMD_READ_CURRENT_TORQUE;        // 0x09
    static const uint8_t CMD_READ_CURRENT;               // 0x04 当前电流值
    static const uint8_t CMD_READ_ERROR_CODE;            // 0x10
    static const uint8_t CMD_READ_MODE_STATUS;           // 0x55 查询执行器当前模式
    static const uint8_t CMD_READ_TARGET_POSITION;       // 0x02
    static const uint8_t CMD_READ_TARGET_SPEED;          // 0x05 目标速度值
    static const uint8_t CMD_READ_TARGET_TORQUE;         // 0x0A
    static const uint8_t CMD_READ_POSITION_DEVIATION;    // 0x03
    static const uint8_t CMD_READ_SPEED_DEVIATION;       // 0x07
    static const uint8_t CMD_READ_TORQUE_DEVIATION;      // 0x0B
    static const uint8_t CMD_READ_CURRENT_POSITION_PULSE; // 0x15
    static const uint8_t CMD_READ_TARGET_POSITION_PULSE;  // 0x16
    static const uint8_t CMD_READ_POSITION_DEVIATION_PULSE; // 0x17
    static const uint8_t CMD_READ_RATED_CURRENT;         // 0x18
    static const uint8_t CMD_READ_RATED_SPEED;           // 0x19
    static const uint8_t CMD_READ_RATED_TORQUE;          // 0x1A
    static const uint8_t CMD_READ_ACTUATOR_ID;           // 0x20
    static const uint8_t CMD_READ_COMMUNICATION_BAUD_RATE; // 0x21
    static const uint8_t CMD_READ_DRIVER_TEMPERATURE;    // 0x22
    static const uint8_t CMD_READ_ACTUATOR_TEMPERATURE;  // 0x23
    static const uint8_t CMD_READ_BUS_VOLTAGE;           // 0x24
    static const uint8_t CMD_READ_PHASE_A_CURRENT;       // 0x25
    static const uint8_t CMD_READ_PHASE_B_CURRENT;       // 0x26
    static const uint8_t CMD_READ_PHASE_C_CURRENT;       // 0x27
    static const uint8_t CMD_READ_ENCODER_POSITION;      // 0x28
    static const uint8_t CMD_READ_ENCODER_SPEED;         // 0x29
    static const uint8_t CMD_READ_CURRENT_KP;            // 0x30
    static const uint8_t CMD_READ_CURRENT_KI;            // 0x31
    static const uint8_t CMD_READ_CURRENT_KD;            // 0x32
    static const uint8_t CMD_READ_SPEED_KP;              // 0x33
    static const uint8_t CMD_READ_SPEED_KI;              // 0x34
    static const uint8_t CMD_READ_SPEED_KD;              // 0x35
    static const uint8_t CMD_READ_POSITION_KP;           // 0x36
    static const uint8_t CMD_READ_POSITION_KI;           // 0x37
    static const uint8_t CMD_READ_POSITION_KD;           // 0x38
    
    static const uint8_t CMD_WRITE_TARGET_POSITION;      // 0x02
    static const uint8_t CMD_WRITE_TARGET_SPEED;         // 0x06
    static const uint8_t CMD_WRITE_TARGET_TORQUE;        // 0x0A
    static const uint8_t CMD_WRITE_ACTUATOR_ID;          // 0x20
    static const uint8_t CMD_WRITE_COMMUNICATION_BAUD_RATE; // 0x21
    static const uint8_t CMD_SET_CURRENT_KP;             // 0x30
    static const uint8_t CMD_SET_CURRENT_KI;             // 0x31
    static const uint8_t CMD_SET_CURRENT_KD;             // 0x32
    static const uint8_t CMD_SET_SPEED_KP;               // 0x33
    static const uint8_t CMD_SET_SPEED_KI;               // 0x34
    static const uint8_t CMD_SET_SPEED_KD;               // 0x35
    static const uint8_t CMD_SET_POSITION_KP;            // 0x36
    static const uint8_t CMD_SET_POSITION_KI;            // 0x37
    static const uint8_t CMD_SET_POSITION_KD;            // 0x38
    static const uint8_t CMD_CLEAR_ERROR;                // 0x10
    static const uint8_t CMD_SAVE_PARAMETERS;            // 0x40
    static const uint8_t CMD_RESTORE_PARAMETERS;         // 0x41
    static const uint8_t CMD_CONTROL_SWITCHING;          // 0x11
    static const uint8_t CMD_MOTOR_SHUTDOWN;             // 0x50
    static const uint8_t CMD_FAULT_SHUTDOWN;             // 0x51
    static const uint8_t CMD_ENABLE_ACTUATOR;            // 0x52
    static const uint8_t CMD_DISABLE_ACTUATOR;           // 0x53
    
    static const uint8_t CMD_HANDSHAKE;                  // 0x44
    static const uint8_t CMD_QUERY_ACTUATOR_ADDRESSES;   // 0x02

};

#endif // MINTASCA_SDK_HPP