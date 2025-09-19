#include "mintasca_sdk.hpp"
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <chrono>

#ifdef _WIN32
using socket_t = SOCKET;
static const socket_t INVALID_SOCKET_FD = INVALID_SOCKET;
#else
using socket_t = int;
static const socket_t INVALID_SOCKET_FD = -1;
#endif

// Define static constants
const std::string MintascaSDK::DEFAULT_IP_ADDRESS = "192.168.1.30";
const uint16_t MintascaSDK::DEFAULT_PORT = 2000;

const uint8_t MintascaSDK::FRAME_HEADER = 0xEE;
const uint8_t MintascaSDK::FRAME_TAIL = 0xED;
const uint8_t MintascaSDK::BROADCAST_ADDRESS = 0x00;

// 定义命令码常量
const uint8_t MintascaSDK::CMD_READ_CURRENT_SPEED = 0x01;        // 当前速度值
const uint8_t MintascaSDK::CMD_READ_CURRENT_POSITION = 0x06;     // 当前位置值
const uint8_t MintascaSDK::CMD_READ_CURRENT_TORQUE = 0x09;
const uint8_t MintascaSDK::CMD_READ_CURRENT = 0x04;              // 当前电流值
const uint8_t MintascaSDK::CMD_READ_ERROR_CODE = 0x10;
const uint8_t MintascaSDK::CMD_READ_MODE_STATUS = 0x55;          // 查询执行器当前模式
const uint8_t MintascaSDK::CMD_READ_TARGET_POSITION = 0x02;
const uint8_t MintascaSDK::CMD_READ_TARGET_SPEED = 0x05;         // 目标速度值
const uint8_t MintascaSDK::CMD_READ_TARGET_TORQUE = 0x0A;
const uint8_t MintascaSDK::CMD_READ_POSITION_DEVIATION = 0x03;
const uint8_t MintascaSDK::CMD_READ_SPEED_DEVIATION = 0x07;
const uint8_t MintascaSDK::CMD_READ_TORQUE_DEVIATION = 0x0B;
const uint8_t MintascaSDK::CMD_READ_CURRENT_POSITION_PULSE = 0x15;
const uint8_t MintascaSDK::CMD_READ_TARGET_POSITION_PULSE = 0x16;
const uint8_t MintascaSDK::CMD_READ_POSITION_DEVIATION_PULSE = 0x17;
const uint8_t MintascaSDK::CMD_READ_RATED_CURRENT = 0x18;
const uint8_t MintascaSDK::CMD_READ_RATED_SPEED = 0x19;
const uint8_t MintascaSDK::CMD_READ_RATED_TORQUE = 0x1A;
const uint8_t MintascaSDK::CMD_READ_ACTUATOR_ID = 0x20;
const uint8_t MintascaSDK::CMD_READ_COMMUNICATION_BAUD_RATE = 0x21;
const uint8_t MintascaSDK::CMD_READ_DRIVER_TEMPERATURE = 0x22;
const uint8_t MintascaSDK::CMD_READ_ACTUATOR_TEMPERATURE = 0x23;
const uint8_t MintascaSDK::CMD_READ_BUS_VOLTAGE = 0x24;
const uint8_t MintascaSDK::CMD_READ_PHASE_A_CURRENT = 0x25;
const uint8_t MintascaSDK::CMD_READ_PHASE_B_CURRENT = 0x26;
const uint8_t MintascaSDK::CMD_READ_PHASE_C_CURRENT = 0x27;
const uint8_t MintascaSDK::CMD_READ_ENCODER_POSITION = 0x28;
const uint8_t MintascaSDK::CMD_READ_ENCODER_SPEED = 0x29;
const uint8_t MintascaSDK::CMD_READ_CURRENT_KP = 0x30;
const uint8_t MintascaSDK::CMD_READ_CURRENT_KI = 0x31;
const uint8_t MintascaSDK::CMD_READ_CURRENT_KD = 0x32;
const uint8_t MintascaSDK::CMD_READ_SPEED_KP = 0x33;
const uint8_t MintascaSDK::CMD_READ_SPEED_KI = 0x34;
const uint8_t MintascaSDK::CMD_READ_SPEED_KD = 0x35;
const uint8_t MintascaSDK::CMD_READ_POSITION_KP = 0x36;
const uint8_t MintascaSDK::CMD_READ_POSITION_KI = 0x37;
const uint8_t MintascaSDK::CMD_READ_POSITION_KD = 0x38;

const uint8_t MintascaSDK::CMD_WRITE_TARGET_POSITION = 0x02;
const uint8_t MintascaSDK::CMD_WRITE_TARGET_SPEED = 0x06;
const uint8_t MintascaSDK::CMD_WRITE_TARGET_TORQUE = 0x0A;
const uint8_t MintascaSDK::CMD_WRITE_ACTUATOR_ID = 0x20;
const uint8_t MintascaSDK::CMD_WRITE_COMMUNICATION_BAUD_RATE = 0x21;
const uint8_t MintascaSDK::CMD_SET_CURRENT_KP = 0x30;
const uint8_t MintascaSDK::CMD_SET_CURRENT_KI = 0x31;
const uint8_t MintascaSDK::CMD_SET_CURRENT_KD = 0x32;
const uint8_t MintascaSDK::CMD_SET_SPEED_KP = 0x33;
const uint8_t MintascaSDK::CMD_SET_SPEED_KI = 0x34;
const uint8_t MintascaSDK::CMD_SET_SPEED_KD = 0x35;
const uint8_t MintascaSDK::CMD_SET_POSITION_KP = 0x36;
const uint8_t MintascaSDK::CMD_SET_POSITION_KI = 0x37;
const uint8_t MintascaSDK::CMD_SET_POSITION_KD = 0x38;
const uint8_t MintascaSDK::CMD_CLEAR_ERROR = 0x10;
const uint8_t MintascaSDK::CMD_SAVE_PARAMETERS = 0x40;
const uint8_t MintascaSDK::CMD_RESTORE_PARAMETERS = 0x41;
const uint8_t MintascaSDK::CMD_CONTROL_SWITCHING = 0x11;
const uint8_t MintascaSDK::CMD_MOTOR_SHUTDOWN = 0x50;
const uint8_t MintascaSDK::CMD_FAULT_SHUTDOWN = 0x51;
const uint8_t MintascaSDK::CMD_ENABLE_ACTUATOR = 0x52;
const uint8_t MintascaSDK::CMD_DISABLE_ACTUATOR = 0x53;

const uint8_t MintascaSDK::CMD_HANDSHAKE = 0x44;
const uint8_t MintascaSDK::CMD_QUERY_ACTUATOR_ADDRESSES = 0x45;

MintascaSDK::MintascaSDK() : port_(0), socket_fd_(INVALID_SOCKET_FD), debug_mode_(false) {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("Failed to initialize Winsock");
    }
#endif
}

MintascaSDK::~MintascaSDK() {
    disconnect();
#ifdef _WIN32
    WSACleanup();
#endif
}

bool MintascaSDK::initialize() {
    return initialize(DEFAULT_IP_ADDRESS, DEFAULT_PORT);
}

bool MintascaSDK::initialize(const std::string& ip_address, uint16_t port) {
    // Store connection parameters
    ip_address_ = ip_address;
    port_ = port;
    
    // Create UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ == INVALID_SOCKET_FD) {
        if (debug_mode_) {
            std::cout << "Failed to create socket" << std::endl;
        }
        return false;
    }
    
    // Setup server address
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
    server_addr_.sin_addr.s_addr = inet_addr(ip_address.c_str());
    
    // Set socket timeout
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    // Perform handshake
    if (!handshake()) {
        if (debug_mode_) {
            std::cout << "Handshake failed" << std::endl;
        }
        disconnect();
        return false;
    }
    
    // Discover actuators
    if (!discoverActuators()) {
        if (debug_mode_) {
            std::cout << "Failed to discover actuators" << std::endl;
        }
        // Continue anyway as we might still be able to communicate with known actuators
    }
    
    if (debug_mode_) {
        std::cout << "Successfully connected to ECB at " << ip_address << ":" << port << std::endl;
    }
    
    return true;
}

bool MintascaSDK::connect() {
    // For UDP, connection is implicit - we just need a valid socket
    return socket_fd_ != INVALID_SOCKET_FD;
}

void MintascaSDK::disconnect() {
    if (socket_fd_ != INVALID_SOCKET_FD) {
#ifdef _WIN32
        closesocket(socket_fd_);
#else
        close(socket_fd_);
#endif
        socket_fd_ = INVALID_SOCKET_FD;
    }
}

bool MintascaSDK::handshake() {
    // Send handshake command: EE 00 44 00 00 ED (without CRC)
    std::vector<uint8_t> handshake_cmd = {FRAME_HEADER, BROADCAST_ADDRESS, CMD_HANDSHAKE, 0x00, 0x00, FRAME_TAIL};
    
    if (debug_mode_) {
        debugPrint("Sending handshake command: ", handshake_cmd);
    }
    
    int sent = sendto(socket_fd_, 
                      reinterpret_cast<const char*>(handshake_cmd.data()), 
                      handshake_cmd.size(), 
                      0, 
                      reinterpret_cast<struct sockaddr*>(&server_addr_), 
                      sizeof(server_addr_));
    
    if (sent != static_cast<int>(handshake_cmd.size())) {
        if (debug_mode_) {
            std::cout << "Failed to send handshake command" << std::endl;
        }
        return false;
    }
    
    // Wait for response
    char buffer[1024];
    struct sockaddr_in response_addr;
    socklen_t response_addr_len = sizeof(response_addr);
    
    int received = recvfrom(socket_fd_, 
                            buffer, 
                            sizeof(buffer), 
                            0, 
                            reinterpret_cast<struct sockaddr*>(&response_addr), 
                            &response_addr_len);
    
    if (received <= 0) {
        if (debug_mode_) {
            std::cout << "Failed to receive handshake response" << std::endl;
        }
        return false;
    }
    
    // Check response
    if (received >= 7 && 
        static_cast<uint8_t>(buffer[0]) == FRAME_HEADER &&
        static_cast<uint8_t>(buffer[1]) == BROADCAST_ADDRESS &&
        static_cast<uint8_t>(buffer[2]) == CMD_HANDSHAKE &&
        static_cast<uint8_t>(buffer[received-1]) == FRAME_TAIL) {
        if (debug_mode_) {
            std::cout << "Handshake successful" << std::endl;
        }
        return true;
    }
    
    if (debug_mode_) {
        std::cout << "Invalid handshake response" << std::endl;
    }
    return false;
}

std::vector<uint8_t> MintascaSDK::queryActuatorAddresses() {
    std::vector<uint8_t> addresses;
    
    // 根据协议，查询执行器命令为: EE 00 02 00 00 ED (without CRC)
    std::vector<uint8_t> query_cmd = {FRAME_HEADER, BROADCAST_ADDRESS, CMD_QUERY_ACTUATOR_ADDRESSES, 0x00, 0x00, FRAME_TAIL};
    
    if (debug_mode_) {
        debugPrint("Sending query actuators command: ", query_cmd);
    }
    
    // Send the command
    int sent = sendto(socket_fd_,
                      reinterpret_cast<const char*>(query_cmd.data()),
                      query_cmd.size(),
                      0,
                      reinterpret_cast<struct sockaddr*>(&server_addr_),
                      sizeof(server_addr_));
    
    if (sent != static_cast<int>(query_cmd.size())) {
        if (debug_mode_) {
            std::cout << "Failed to send query actuators command" << std::endl;
        }
        return addresses;
    }
    
    // Wait for response
    char buffer[1024];
    struct sockaddr_in response_addr;
    socklen_t response_addr_len = sizeof(response_addr);
    
    // 设置接收超时（查询执行器需要更长时间，因为要轮询查找执行器）
    struct timeval timeout;
    timeout.tv_sec = 1;  // 1秒超时
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    // 循环接收响应，最多等待1秒以接收所有执行器的响应
    int total_received = 0;
    auto start_time = std::chrono::steady_clock::now();
    const auto max_wait_time = std::chrono::milliseconds(1000); // 最多等待1秒
    
    while (std::chrono::steady_clock::now() - start_time < max_wait_time) {
        int received = recvfrom(socket_fd_,
                                buffer + total_received,
                                sizeof(buffer) - total_received,
                                0,
                                reinterpret_cast<struct sockaddr*>(&response_addr),
                                &response_addr_len);
        
        if (received > 0) {
            total_received += received;
            
            // 如果缓冲区接近满，停止接收
            if (total_received >= static_cast<int>(sizeof(buffer) - 10)) {
                break;
            }
        } else if (received == 0) {
            // 连接关闭
            break;
        }
        // received < 0 表示超时，继续循环直到达到最大等待时间
    }
    
    if (debug_mode_) {
        std::cout << "Received " << total_received << " bytes in total after waiting" << std::endl;
        std::vector<uint8_t> response_data(reinterpret_cast<uint8_t*>(buffer), 
                                          reinterpret_cast<uint8_t*>(buffer) + total_received);
        debugPrint("Raw response data: ", response_data);
    }
    
    // 解析响应以提取所有执行器地址和序列号
    int index = 0;
    while (index < total_received - 6) { // 至少需要6个字节组成一个最小帧
        // 查找帧头
        while (index < total_received - 6 && static_cast<uint8_t>(buffer[index]) != FRAME_HEADER) {
            index++;
        }
        
        if (index >= total_received - 6) break;
        
        // 解析帧的基本信息
        uint8_t header = static_cast<uint8_t>(buffer[index]);
        uint8_t address = static_cast<uint8_t>(buffer[index + 1]);
        uint8_t cmd = static_cast<uint8_t>(buffer[index + 2]);
        uint16_t data_length = (static_cast<uint8_t>(buffer[index + 3]) << 8) | static_cast<uint8_t>(buffer[index + 4]);
        
        // 计算帧的总长度
        int frame_length = 5 + data_length + (data_length > 0 ? 2 : 0) + 1; // header(1) + addr(1) + cmd(1) + len(2) + data + CRC(0/2) + tail(1)
        
        // 检查是否有足够的数据组成完整帧
        if (index + frame_length > total_received) {
            index++;
            continue;
        }
        
        uint8_t tail = static_cast<uint8_t>(buffer[index + frame_length - 1]);
        
        if (header == FRAME_HEADER && tail == FRAME_TAIL && cmd == CMD_QUERY_ACTUATOR_ADDRESSES) {
            // 提取执行器ID
            addresses.push_back(address);
            
            if (debug_mode_) {
                std::cout << "Found actuator with ID: " << static_cast<int>(address) << std::endl;
                
                // 提取序列号（如果存在）
                if (data_length >= 4) {
                    uint32_t serial = (static_cast<uint32_t>(static_cast<uint8_t>(buffer[index + 5])) << 24) |
                                      (static_cast<uint32_t>(static_cast<uint8_t>(buffer[index + 6])) << 16) |
                                      (static_cast<uint32_t>(static_cast<uint8_t>(buffer[index + 7])) << 8) |
                                      static_cast<uint32_t>(static_cast<uint8_t>(buffer[index + 8]));
                    std::cout << "Actuator " << static_cast<int>(address) << " serial number: " << serial << std::endl;
                }
            }
            
            // 移动到下一帧
            index += frame_length;
        } else {
            index++;
        }
    }
    
    return addresses;
}

std::vector<uint8_t> MintascaSDK::testQueryActuatorAddresses() {
    std::vector<uint8_t> addresses;
    
    // 根据协议，查询执行器命令为: EE 00 02 00 00 ED
    const uint8_t QUERY_ACTUATORS_CMD = 0x02;
    std::vector<uint8_t> query_cmd = {FRAME_HEADER, BROADCAST_ADDRESS, QUERY_ACTUATORS_CMD, 0x00, 0x00, FRAME_TAIL};
    
    std::cout << "Sending query actuators command (0x02): ";
    for (size_t i = 0; i < query_cmd.size(); ++i) {
        printf("%02X ", query_cmd[i]);
    }
    printf("\n");
    
    // Send the command
    int sent = sendto(socket_fd_,
                      reinterpret_cast<const char*>(query_cmd.data()),
                      query_cmd.size(),
                      0,
                      reinterpret_cast<struct sockaddr*>(&server_addr_),
                      sizeof(server_addr_));
    
    if (sent != static_cast<int>(query_cmd.size())) {
        std::cout << "Failed to send query actuators command" << std::endl;
        return addresses;
    }
    
    std::cout << "Query command sent successfully" << std::endl;
    
    // Wait for response
    char buffer[1024];
    struct sockaddr_in response_addr;
    socklen_t response_addr_len = sizeof(response_addr);
    
    // 设置接收超时（查询执行器可能需要更长时间）
    struct timeval timeout;
    timeout.tv_sec = 3;  // 3秒超时
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    std::cout << "Waiting for response (up to 3 seconds)..." << std::endl;
    
    // 可能收到多个响应，需要循环接收
    int total_received = 0;
    while (true) {
        int received = recvfrom(socket_fd_,
                                buffer + total_received,
                                sizeof(buffer) - total_received,
                                0,
                                reinterpret_cast<struct sockaddr*>(&response_addr),
                                &response_addr_len);
        
        if (received <= 0) {
            // 超时或无更多数据
            if (total_received == 0) {
                std::cout << "Failed to receive response or timeout" << std::endl;
            }
            break;
        }
        
        total_received += received;
        
        // 检查是否已收到完整帧
        if (total_received >= 6 && 
            static_cast<uint8_t>(buffer[total_received-1]) == FRAME_TAIL) {
            break;
        }
        
        // 如果缓冲区满了，停止接收
        if (total_received >= static_cast<int>(sizeof(buffer))) {
            break;
        }
    }
    
    std::cout << "Received " << total_received << " bytes in total" << std::endl;
    for (int i = 0; i < total_received; i++) {
        printf("%02X ", static_cast<unsigned char>(buffer[i]));
    }
    printf("\n");
    
    // 解析响应以提取地址和序列号
    int index = 0;
    while (index < total_received) {
        // 查找帧头
        while (index < total_received && static_cast<uint8_t>(buffer[index]) != FRAME_HEADER) {
            index++;
        }
        
        if (index >= total_received) break;
        
        // 确保有足够的数据用于解析
        if (index + 6 > total_received) break;
        
        // 解析帧
        uint8_t header = static_cast<uint8_t>(buffer[index]);
        uint8_t address = static_cast<uint8_t>(buffer[index + 1]);
        uint8_t cmd = static_cast<uint8_t>(buffer[index + 2]);
        uint16_t data_length = (static_cast<uint8_t>(buffer[index + 3]) << 8) | static_cast<uint8_t>(buffer[index + 4]);
        uint8_t tail = static_cast<uint8_t>(buffer[index + 5 + data_length]);
        
        if (header == FRAME_HEADER && tail == FRAME_TAIL && cmd == QUERY_ACTUATORS_CMD) {
            // 提取执行器ID
            addresses.push_back(address);
            
            std::cout << "Found actuator with ID: " << static_cast<int>(address) << std::endl;
            
            // 提取序列号（如果存在）
            if (data_length >= 4) {
                uint32_t serial = (static_cast<uint32_t>(static_cast<uint8_t>(buffer[index + 5])) << 24) |
                                  (static_cast<uint32_t>(static_cast<uint8_t>(buffer[index + 6])) << 16) |
                                  (static_cast<uint32_t>(static_cast<uint8_t>(buffer[index + 7])) << 8) |
                                  static_cast<uint32_t>(static_cast<uint8_t>(buffer[index + 8]));
                std::cout << "Actuator " << static_cast<int>(address) << " serial number: " << serial << std::endl;
            }
            
            // 移动到下一帧
            index += 6 + data_length;
        } else {
            index++;
        }
    }
    
    return addresses;
}

bool MintascaSDK::discoverActuators() {
    // Clear previous actuator information
    actuators_.clear();
    
    // Query actuator addresses
    std::vector<uint8_t> addresses = queryActuatorAddresses();
    
    if (addresses.empty()) {
        if (debug_mode_) {
            std::cout << "No actuators found during discovery" << std::endl;
        }
        return false;
    }
    
    // For each discovered actuator, get its information
    for (uint8_t addr : addresses) {
        if (debug_mode_) {
            std::cout << "Getting information for actuator " << static_cast<int>(addr) << std::endl;
        }
        
        // Get current position (using correct command code 0x06)
        float position = readCurrentPosition(addr);
        
        // Get mode status (using correct command code 0x55)
        uint8_t mode = readModeStatus(addr);
        
        // Create actuator info with serial number if we can extract it
        std::string serial = "SN-" + std::to_string(addr);
        actuators_[addr] = ActuatorInfo(addr, serial, position, mode);
    }
    
    return true;
}


// Helper methods
void MintascaSDK::debugPrint(const std::string& message, const std::vector<uint8_t>& data) {
    if (!debug_mode_) return;
    
    std::cout << "[DEBUG] " << message << std::endl;
    std::cout << "Hex dump: ";
    for (size_t i = 0; i < data.size(); ++i) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    
    // Add ASCII representation for better readability
    std::cout << "ASCII:  ";
    for (size_t i = 0; i < data.size(); ++i) {
        char c = (data[i] >= 32 && data[i] <= 126) ? data[i] : '.';
        printf("%c", c);
    }
    printf("\n\n");
}

std::vector<uint8_t> MintascaSDK::sendCommand(uint8_t address, uint8_t command, 
                                              const std::vector<uint8_t>& data) {
    std::vector<uint8_t> response;
    
    // Build command frame
    std::vector<uint8_t> frame;
    frame.push_back(FRAME_HEADER);
    frame.push_back(address);
    frame.push_back(command);
    
    // Add data length (2 bytes, big endian)
    uint16_t data_length = data.size();
    frame.push_back((data_length >> 8) & 0xFF);
    frame.push_back(data_length & 0xFF);
    
    // Add data
    frame.insert(frame.end(), data.begin(), data.end());
    
    // Add CRC only if data exists
    // 特定命令不需要CRC: 0x44 (握手)、0x02 (查询执行器)
    if (!data.empty() && command != CMD_HANDSHAKE && command != CMD_QUERY_ACTUATOR_ADDRESSES) {
        uint16_t crc = calculateCRC(data);  // CRC只计算数据内容
        frame.push_back((crc >> 8) & 0xFF);
        frame.push_back(crc & 0xFF);
    }
    
    // Add frame tail
    frame.push_back(FRAME_TAIL);
    
    // Debug print sent data with more context
    if (debug_mode_) {
        char cmd_context[256];
        snprintf(cmd_context, sizeof(cmd_context), "Sending command (addr=0x%02X, cmd=0x%02X): ", address, command);
        debugPrint(std::string(cmd_context), frame);
    }
    
    // Send command
    int sent = sendto(socket_fd_,
                      reinterpret_cast<const char*>(frame.data()),
                      frame.size(),
                      0,
                      reinterpret_cast<struct sockaddr*>(&server_addr_),
                      sizeof(server_addr_));
    
    if (debug_mode_) {
        std::cout << "Sent " << sent << " bytes" << std::endl;
    }
    
    if (sent != static_cast<int>(frame.size())) {
        return response;
    }
    
    // Wait for response
    char buffer[1024];
    struct sockaddr_in response_addr;
    socklen_t response_addr_len = sizeof(response_addr);
    
    // 设置接收超时
    struct timeval timeout;
    timeout.tv_sec = 2;  // 2秒超时
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    int received = recvfrom(socket_fd_,
                            buffer,
                            sizeof(buffer),
                            0,
                            reinterpret_cast<struct sockaddr*>(&response_addr),
                            &response_addr_len);
    
    if (debug_mode_) {
        std::cout << "Received " << received << " bytes" << std::endl;
        if (received > 0) {
            std::cout << "First byte: 0x" << std::hex << static_cast<int>(buffer[0]) << std::dec << std::endl;
            std::cout << "Command byte: 0x" << std::hex << static_cast<int>(buffer[2]) << std::dec << std::endl;
            
            std::vector<uint8_t> raw_data(reinterpret_cast<uint8_t*>(buffer), 
                                        reinterpret_cast<uint8_t*>(buffer) + received);
            debugPrint("Raw response data: ", raw_data);
        }
    }
    
    if (received > 0) {
        response.resize(received);
        for (int i = 0; i < received; i++) {
            response[i] = static_cast<uint8_t>(buffer[i]);
        }
    }
    
    return response;
}

uint16_t MintascaSDK::calculateCRC(const std::vector<uint8_t>& data) {
    // Simple CRC implementation - in a real implementation, this should follow
    // the specific CRC16 algorithm mentioned in the protocol documentation
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> MintascaSDK::floatToIQ24(float value) {
    // Convert float to IQ24 format (fixed point with 24 fractional bits)
    int32_t iq24 = static_cast<int32_t>(value * (1 << 24));
    
    // Convert to big endian bytes
    std::vector<uint8_t> bytes(4);
    bytes[0] = (iq24 >> 24) & 0xFF;
    bytes[1] = (iq24 >> 16) & 0xFF;
    bytes[2] = (iq24 >> 8) & 0xFF;
    bytes[3] = iq24 & 0xFF;
    
    return bytes;
}

float MintascaSDK::IQ24ToFloat(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        return 0.0f;
    }
    
    // Convert big endian bytes to IQ24
    int32_t iq24 = (static_cast<int32_t>(data[0]) << 24) |
                   (static_cast<int32_t>(data[1]) << 16) |
                   (static_cast<int32_t>(data[2]) << 8) |
                   static_cast<int32_t>(data[3]);
    
    // Convert IQ24 to float
    return static_cast<float>(iq24) / (1 << 24);
}

std::vector<uint8_t> MintascaSDK::uint32ToBytes(uint32_t value) {
    std::vector<uint8_t> bytes(4);
    bytes[0] = (value >> 24) & 0xFF;
    bytes[1] = (value >> 16) & 0xFF;
    bytes[2] = (value >> 8) & 0xFF;
    bytes[3] = value & 0xFF;
    return bytes;
}

uint32_t MintascaSDK::bytesToUint32(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        return 0;
    }
    
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

uint8_t MintascaSDK::bytesToUint8(const std::vector<uint8_t>& data) {
    if (data.empty()) {
        return 0;
    }
    
    return data[0];
}

// Read methods
float MintascaSDK::readCurrentSpeed(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_CURRENT_SPEED);
    
    // Parse response: EE [addr] 05 [data len] [4 bytes data] [2 bytes CRC] ED
    // 0x05: 当前速度值，速度真实值需要乘以速度满量程，单位为RPM
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_CURRENT_SPEED && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readCurrentPosition(uint8_t actuator_id) {
    if (debug_mode_) {
        std::cout << "readCurrentPosition called for actuator 0x" 
                  << std::hex << static_cast<int>(actuator_id) << std::dec << std::endl;
    }
    
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_CURRENT_POSITION);
    
    if (debug_mode_ && !response.empty()) {
        std::cout << "Response size: " << response.size() << " bytes" << std::endl;
        std::cout << "Response[0]: 0x" << std::hex << static_cast<int>(response[0]) << std::dec << std::endl;
        std::cout << "Response[2]: 0x" << std::hex << static_cast<int>(response[2]) << std::dec << std::endl;
        std::cout << "Response[last]: 0x" << std::hex << static_cast<int>(response.back()) << std::dec << std::endl;
    }
    
    // Parse response: EE [addr] 06 [data len] [4 bytes data] [2 bytes CRC] ED
    // 0x06: 当前位置值，单位为R
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_CURRENT_POSITION && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        float result = IQ24ToFloat(data);
        
        if (debug_mode_) {
            std::cout << "Current position: " << result << std::endl;
        }
        
        return result;
    }
    
    if (debug_mode_) {
        std::cout << "Failed to read current position" << std::endl;
    }
    
    return 0.0f;
}

uint8_t MintascaSDK::readModeStatus(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_MODE_STATUS);
    
    // Parse response
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_MODE_STATUS && 
        response[response.size()-1] == FRAME_TAIL) {
        
        // Data length should be 1 byte for mode status
        return response[5];
    }
    
    return 0;
}

float MintascaSDK::readCurrent(uint8_t address) {
    std::vector<uint8_t> response = sendCommand(address, CMD_READ_CURRENT);
    if (response.size() >= 9) {
        // 提取IQ24格式的数据
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    return 0.0f;

}

float MintascaSDK::readCurrentTorque(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_CURRENT_TORQUE);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_CURRENT_TORQUE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

uint32_t MintascaSDK::readErrorCode(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_ERROR_CODE);
    
    if (response.size() >= 9 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_ERROR_CODE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return bytesToUint32(data);
    }
    
    return 0;
}

float MintascaSDK::readTargetPosition(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_TARGET_POSITION);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_TARGET_POSITION && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readTargetSpeed(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_TARGET_SPEED);
    
    // Parse response: EE [addr] 05 [data len] [4 bytes data] [2 bytes CRC] ED
    // 0x05: 当前速度值，速度真实值需要乘以速度满量程，单位为RPM
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_TARGET_SPEED && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readTargetTorque(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_TARGET_TORQUE);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_TARGET_TORQUE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readPositionDeviation(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_POSITION_DEVIATION);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_POSITION_DEVIATION && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readSpeedDeviation(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_SPEED_DEVIATION);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_SPEED_DEVIATION && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readTorqueDeviation(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_TORQUE_DEVIATION);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_TORQUE_DEVIATION && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

uint32_t MintascaSDK::readCurrentPositionPulse(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_CURRENT_POSITION_PULSE);
    
    if (response.size() >= 9 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_CURRENT_POSITION_PULSE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return bytesToUint32(data);
    }
    
    return 0;
}

uint32_t MintascaSDK::readTargetPositionPulse(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_TARGET_POSITION_PULSE);
    
    if (response.size() >= 9 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_TARGET_POSITION_PULSE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return bytesToUint32(data);
    }
    
    return 0;
}

uint32_t MintascaSDK::readPositionDeviationPulse(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_POSITION_DEVIATION_PULSE);
    
    if (response.size() >= 9 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_POSITION_DEVIATION_PULSE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return bytesToUint32(data);
    }
    
    return 0;
}

float MintascaSDK::readRatedCurrent(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_RATED_CURRENT);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_RATED_CURRENT && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readRatedSpeed(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_RATED_SPEED);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_RATED_SPEED && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readRatedTorque(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_RATED_TORQUE);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_RATED_TORQUE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

uint8_t MintascaSDK::readActuatorID(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_ACTUATOR_ID);
    
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_ACTUATOR_ID && 
        response[response.size()-1] == FRAME_TAIL) {
        
        return response[5];
    }
    
    return 0;
}

uint32_t MintascaSDK::readCommunicationBaudRate(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_COMMUNICATION_BAUD_RATE);
    
    if (response.size() >= 9 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_COMMUNICATION_BAUD_RATE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return bytesToUint32(data);
    }
    
    return 0;
}

uint8_t MintascaSDK::readDriverTemperature(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_DRIVER_TEMPERATURE);
    
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_DRIVER_TEMPERATURE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        return response[5];
    }
    
    return 0;
}

uint8_t MintascaSDK::readActuatorTemperature(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_ACTUATOR_TEMPERATURE);
    
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_ACTUATOR_TEMPERATURE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        return response[5];
    }
    
    return 0;
}

float MintascaSDK::readBusVoltage(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_BUS_VOLTAGE);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_BUS_VOLTAGE && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readPhaseACurrent(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_PHASE_A_CURRENT);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_PHASE_A_CURRENT && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readPhaseBCurrent(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_PHASE_B_CURRENT);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_PHASE_B_CURRENT && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readPhaseCCurrent(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_PHASE_C_CURRENT);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_PHASE_C_CURRENT && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

uint32_t MintascaSDK::readEncoderPosition(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_ENCODER_POSITION);
    
    if (response.size() >= 9 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_ENCODER_POSITION && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return bytesToUint32(data);
    }
    
    return 0;
}

uint32_t MintascaSDK::readEncoderSpeed(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_ENCODER_SPEED);
    
    if (response.size() >= 9 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_ENCODER_SPEED && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return bytesToUint32(data);
    }
    
    return 0;
}

float MintascaSDK::readCurrentKp(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_CURRENT_KP);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_CURRENT_KP && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readCurrentKi(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_CURRENT_KI);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_CURRENT_KI && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readCurrentKd(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_CURRENT_KD);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_CURRENT_KD && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readSpeedKp(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_SPEED_KP);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_SPEED_KP && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readSpeedKi(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_SPEED_KI);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_SPEED_KI && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readSpeedKd(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_SPEED_KD);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_SPEED_KD && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readPositionKp(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_POSITION_KP);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_POSITION_KP && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readPositionKi(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_POSITION_KI);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_POSITION_KI && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

float MintascaSDK::readPositionKd(uint8_t actuator_id) {
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_READ_POSITION_KD);
    
    if (response.size() >= 10 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_READ_POSITION_KD && 
        response[response.size()-1] == FRAME_TAIL) {
        
        std::vector<uint8_t> data(response.begin() + 5, response.begin() + 9);
        return IQ24ToFloat(data);
    }
    
    return 0.0f;
}

// Write methods
bool MintascaSDK::writeTargetPosition(uint8_t actuator_id, float position) {
    std::vector<uint8_t> data = floatToIQ24(position);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_WRITE_TARGET_POSITION, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_WRITE_TARGET_POSITION && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::writeTargetSpeed(uint8_t actuator_id, float speed) {
    std::vector<uint8_t> data = floatToIQ24(speed);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_WRITE_TARGET_SPEED, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_WRITE_TARGET_SPEED && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::writeTargetTorque(uint8_t actuator_id, float torque) {
    std::vector<uint8_t> data = floatToIQ24(torque);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_WRITE_TARGET_TORQUE, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_WRITE_TARGET_TORQUE && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::writeActuatorID(uint8_t actuator_id, uint8_t new_id) {
    std::vector<uint8_t> data(1, new_id);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_WRITE_ACTUATOR_ID, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_WRITE_ACTUATOR_ID && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::writeCommunicationBaudRate(uint8_t actuator_id, uint32_t baud_rate) {
    std::vector<uint8_t> data = uint32ToBytes(baud_rate);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_WRITE_COMMUNICATION_BAUD_RATE, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_WRITE_COMMUNICATION_BAUD_RATE && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setCurrentKp(uint8_t actuator_id, float kp) {
    std::vector<uint8_t> data = floatToIQ24(kp);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_CURRENT_KP, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_CURRENT_KP && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setCurrentKi(uint8_t actuator_id, float ki) {
    std::vector<uint8_t> data = floatToIQ24(ki);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_CURRENT_KI, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_CURRENT_KI && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setCurrentKd(uint8_t actuator_id, float kd) {
    std::vector<uint8_t> data = floatToIQ24(kd);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_CURRENT_KD, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_CURRENT_KD && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setSpeedKp(uint8_t actuator_id, float kp) {
    std::vector<uint8_t> data = floatToIQ24(kp);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_SPEED_KP, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_SPEED_KP && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setSpeedKi(uint8_t actuator_id, float ki) {
    std::vector<uint8_t> data = floatToIQ24(ki);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_SPEED_KI, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_SPEED_KI && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setSpeedKd(uint8_t actuator_id, float kd) {
    std::vector<uint8_t> data = floatToIQ24(kd);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_SPEED_KD, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_SPEED_KD && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setPositionKp(uint8_t actuator_id, float kp) {
    std::vector<uint8_t> data = floatToIQ24(kp);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_POSITION_KP, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_POSITION_KP && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setPositionKi(uint8_t actuator_id, float ki) {
    std::vector<uint8_t> data = floatToIQ24(ki);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_POSITION_KI, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_POSITION_KI && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::setPositionKd(uint8_t actuator_id, float kd) {
    std::vector<uint8_t> data = floatToIQ24(kd);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SET_POSITION_KD, data);
    
    // Check if write was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SET_POSITION_KD && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::clearError(uint8_t actuator_id) {
    std::vector<uint8_t> data(1, 0x00); // No data needed
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_CLEAR_ERROR, data);
    
    // Check if command was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_CLEAR_ERROR && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::saveParameters(uint8_t actuator_id) {
    std::vector<uint8_t> data(1, 0x00); // No data needed
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_SAVE_PARAMETERS, data);
    
    // Check if command was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_SAVE_PARAMETERS && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::restoreParameters(uint8_t actuator_id) {
    std::vector<uint8_t> data(1, 0x00); // No data needed
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_RESTORE_PARAMETERS, data);
    
    // Check if command was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_RESTORE_PARAMETERS && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::controlSwitching(uint8_t actuator_id, uint8_t mode) {
    std::vector<uint8_t> data(1, mode);
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_CONTROL_SWITCHING, data);
    
    // Check if command was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_CONTROL_SWITCHING && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::motorShutdown(uint8_t actuator_id) {
    std::vector<uint8_t> data(1, 0x00); // No data needed
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_MOTOR_SHUTDOWN, data);
    
    // Check if command was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_MOTOR_SHUTDOWN && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::faultShutdown(uint8_t actuator_id) {
    std::vector<uint8_t> data(1, 0x00); // No data needed
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_FAULT_SHUTDOWN, data);
    
    // Check if command was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_FAULT_SHUTDOWN && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::enableActuator(uint8_t actuator_id) {
    std::vector<uint8_t> data(1, 0x01); // Enable command
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_ENABLE_ACTUATOR, data);
    
    // Check if command was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_ENABLE_ACTUATOR && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

bool MintascaSDK::disableActuator(uint8_t actuator_id) {
    std::vector<uint8_t> data(1, 0x00); // Disable command
    std::vector<uint8_t> response = sendCommand(actuator_id, CMD_DISABLE_ACTUATOR, data);
    
    // Check if command was successful
    if (response.size() >= 7 && 
        response[0] == FRAME_HEADER && 
        response[2] == CMD_DISABLE_ACTUATOR && 
        response[response.size()-1] == FRAME_TAIL && 
        response[5] == 0x01) {
        return true;
    }
    
    return false;
}

// Additional read/write methods would be implemented similarly...
// For brevity, I'm only implementing a few key ones above
// The full implementation would include all methods declared in the header