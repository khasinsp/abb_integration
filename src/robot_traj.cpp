#include <bits/stdc++.h>
#include "../include/tcp_server.h"
#include "../include/command.h"

#define LOCAL_HOST "192.168.0.60"
#define LOCAL_PORT_RX 5001
#define LOCAL_PORT_TX 5000

#define ANGLE_THRESHOLD 360
#define SPEED_THRESHOLD 200
#define FREQ 100
#define DELAY 5

#define A_MAX 1.0

struct RobotState {
    float react_timestamp;
    uint32_t queue_size;
    float values[6];
};

RobotState unpack(const std::string& buffer) {
    RobotState state;

    if (buffer.size() < sizeof(RobotState)) {
        std::cout << "response too short" << std::endl;
        // TODO (What should happen here?)
    }

    std::memcpy(&state, buffer.data(), sizeof(RobotState));

    return state;
}

bool check_buffer(const std::string &buffer) {
    if (buffer.size() < 2) return false;

    return static_cast<unsigned char>(buffer[buffer.size() - 2]) == 0xAA &&
           static_cast<unsigned char>(buffer[buffer.size() - 1]) == 0x55;
}

std::queue<std::vector<float>> command_arr;

float t_end;

void generate_motion(std::queue<std::vector<float>> &command_arr, std::vector<float> command) {

    // calculate length of command array given frequency and speed
    float speed = command.back();
    command.pop_back();
    float max_angle = *std::max_element(command.begin(), command.end());

    t_end = max_angle / speed;
    std::cout << "Max Angle is: " << max_angle << std::endl;
    int commandlen = max_angle / speed * FREQ;

    for (int i = 0; i < commandlen; i++) {
        std::vector<float> c(command.size());
        for (int j = 0; j < c.size(); j++) {
            c[j] = command[j] / (commandlen - 1) * i;
            
        }
        command_arr.push(c);
    }

    return;

}

void user_input_thread() {

    while (true) {
        float user_input;
        std::vector<float> command;
        std::cout << "Type:\nangle0 angle1 angle2 angle3 angle4 angle5 angle6 speed[deg/s]" << std::endl;
        for (int i = 0; i < 7; i++) {
            std::cin >> user_input;
            command.push_back(user_input);
        }

        bool high_angle = false;
        bool high_speed = false;
        for (int i = 0; i < command.size(); i++) {
            if (i < command.size() - 1) {
                if (command[i] > ANGLE_THRESHOLD) {
                    std::cout << "high angle: joint " << i+1 << std::endl;
                    high_angle = true;
                }
            }
            if (i == command.size() - 1) {
                if (command[i] > SPEED_THRESHOLD) {
                    std::cout << "high speed: " << command[i] << std::endl;
                    high_speed = true;
                }
            }
        }

        if (high_angle || high_speed) {
            std::cout << "Invalid value, try again!" << std::endl;
            continue;
        }

        generate_motion(command_arr, command);
    }
}

std::vector<uint8_t> generate_command(std::vector<float> command, float s) {
    std::vector<uint8_t> buffer;
    const uint8_t end_of_command[] = { 0xAA, 0x55 };
    uint32_t last_value = 10;
    uint8_t last_value_bytes [sizeof(uint32_t)];
    std::memcpy(last_value_bytes, &last_value, sizeof(uint32_t));

    // Copy Joint values to send buffer
    for (float joint : command) {
        uint8_t bytes[sizeof(float)];
        std::memcpy(bytes, &joint, sizeof(float));
        buffer.insert(buffer.end(), bytes, bytes + sizeof(float));
    }

    // Copy Start time to send buffer
    uint8_t timeBytes[sizeof(float)];
    std::memcpy(timeBytes, &s, sizeof(float));
    buffer.insert(buffer.end(), timeBytes, timeBytes + sizeof(float));

    // Copy last value bytes (10) to send buffer
    buffer.insert(buffer.end(), last_value_bytes, last_value_bytes + sizeof(uint32_t));
    
    // Copy End of Command bytes (0xAA, 0x55) to send buffer
    buffer.insert(buffer.end(), std::begin(end_of_command), std::end(end_of_command));

    return buffer;
}


void main_thread() {
    TCPServer *socketServerTX = new TCPServer(LOCAL_HOST, LOCAL_PORT_TX);
    TCPServer *socketServerRX = new TCPServer(LOCAL_HOST, LOCAL_PORT_RX);

    socketServerTX->start_();
    socketServerRX->start_();

    std::string in_buffer;
    std::string received_main_buffer;

    std::vector<uint8_t> out_buffer;
    
    auto start = std::chrono::high_resolution_clock::now();
    while (true) {

        ssize_t received_bytes = socketServerRX->recv_(in_buffer);
        if (received_bytes > 0) {
            received_main_buffer += in_buffer;
        }

        if (check_buffer(received_main_buffer)) {
            received_main_buffer.pop_back();
            received_main_buffer.pop_back();
            RobotState current_state = unpack(received_main_buffer);
            float end_time = current_state.react_timestamp+DELAY;
            received_main_buffer = "";
            if (command_arr.size() > 0) {
                std::vector<float> command = command_arr.front();
                for (float c : command) {
                    std::cout << c << ", ";
                }
                std::cout << std::endl;
                command_arr.pop();
                out_buffer = generate_command(command, 0.01);
                ssize_t sent_bytes = socketServerTX->send_(out_buffer);
            }
            else {
                std::vector<float> command(std::begin(current_state.values), std::end(current_state.values));
                out_buffer = generate_command(command, end_time);
                ssize_t sent_bytes = socketServerTX->send_(out_buffer);
            }

            auto end = std::chrono::high_resolution_clock::now();
            int loop_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << loop_time << std::endl;
            start = std::chrono::high_resolution_clock::now();
        }
    }
}

int main(int argc, char **argv) {

    std::thread main_t(main_thread);
    std::thread user_input_t(user_input_thread);

    main_t.join();
    user_input_t.join();

    return 0;

}