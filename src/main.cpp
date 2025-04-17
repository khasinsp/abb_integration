#include <bits/stdc++.h>
#include "../include/tcp_server.h"
#include "../include/command.h"

#define LOCAL_HOST "192.168.0.60"
#define LOCAL_PORT_RX 5001
#define LOCAL_PORT_TX 5000

#define ANGLE_THRESHOLD 5
#define SPEED_THRESHOLD 10
#define FREQ 100

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


RobotState current_pose;
float start_delay = 0;
float start_time;

void listener_thread() {

    TCPServer *socketServerRX = new TCPServer(LOCAL_HOST, LOCAL_PORT_RX);
    socketServerRX->start_();

    std::string main_buffer;
    std::string in_buffer;

    unsigned short i = 0;
    while (true) {
        ssize_t bytes = socketServerRX->recv_(in_buffer);
        if (bytes < 0) {
            continue;
        }
        if (bytes > 0) {
            main_buffer += in_buffer;
        }
        if (bytes == 0) {
            std::cout << "Error in recv_() in listener_thread()" << std::endl;
            exit(EXIT_FAILURE);
        }

        if (check_buffer(main_buffer)) {
            current_pose = unpack(main_buffer);
            main_buffer = "";
            start_delay = current_pose.react_timestamp + 5;
            std::cout << "start delay: " << start_delay << std::endl;
            // for (int i = 0; i < 6; i++) {
            //     std::cout << current_pose.values[i] << ", ";
            // } 
            // std::cout << std::endl;
            // if (current_pose.queue_size > 0) std::cout << "queue size: " << current_pose.queue_size << std::endl;
        }
    }
}

void generate_motion(std::vector<std::vector<float>> &command_arr, std::vector<float> command) {

    // calculate length of command array given frequency and speed
    float speed = command.back();
    command.pop_back();
    float max_angle = *std::max_element(command.begin(), command.end());
    std::cout << "Max Angle is: " << max_angle << std::endl;
    int commandlen = max_angle / speed * FREQ;

    // We want joint correction given the current joint angles
    // first vector in command are current joint angles
    // if (command_arr.size() == 0) {
    //     std::vector<float> init_vec;
    //     for (int i = 0; i < command.size(); i++) {
    //         init_vec.push_back(current_pose.values[i]);
    //     }
    //     command_arr.push_back(init_vec);
    // }
    // // if robot moving (command array not empty) and new command given -> wait until command array is empty
    // else {
    //     while (command_arr.size() > 0) {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     }
    //     std::vector<float> init_vec;
    //     for (int i = 0; i < command.size(); i++) {
    //         init_vec.push_back(current_pose.values[i]);
    //     }
    //     command_arr.push_back(init_vec);
    // }

    for (int i = 0; i < commandlen; i++) {
        std::vector<float> c(command.size());
        for (int j = 0; j < c.size(); j++) {
            // c[j] = command_arr[0][j] + command[j] / (commandlen - 1) * i;
            c[j] = command[j] / (commandlen - 1) * i;
            // std::cout << c[j] << ", ";
        }
        // std::cout << std::endl;
        command_arr.push_back(c);
    }

    return;

}

std::vector<std::vector<float>> command_arr;



void command_thread() {

    while (true) {
        float user_input;
        std::vector<float> command;
        std::cout << "Type:\nangle0 angle1 angle2 angle3 angle4 angle5 angle6 speed[deg/s]" << std::endl;
        for (int i = 0; i < 7; i++) {
            std::cin >> user_input;
            command.push_back(user_input);
        }

        if (command.size() != 7) {
            std::cout << "Wrong command size, try again!" << std::endl;
            continue;
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

        start_time = start_delay;
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

void settingPose_thread() {

    TCPServer *socketServerTX = new TCPServer(LOCAL_HOST, LOCAL_PORT_TX);
    socketServerTX->start_();

    std::string main_command_buffer;
    std::string in_command_buffer;

    const uint8_t end_of_command[] = { 0xAA, 0x55 };
    uint32_t last_value = 10;
    uint8_t last_value_bytes [sizeof(uint32_t)];
    std::memcpy(last_value_bytes, &last_value, sizeof(uint32_t));

    int i = 0;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        if (command_arr.size() > 0) {
            float s = start_time + i*0.01;
            // std::cout << s << std::endl;

            std::vector<float> c = command_arr.front();
            command_arr.erase(command_arr.begin());

            std::vector<uint8_t> buffer;

            buffer = generate_command(c, s);

            // for (uint8_t byte : buffer) {
            //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            // }
            // std::cout << std::endl;

            ssize_t send_bytes = socketServerTX->send_(buffer);

            i++;
            
        }

        if (current_pose.queue_size > 0) {
            auto end = std::chrono::high_resolution_clock::now();
            auto micro = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            std::this_thread::sleep_for(std::chrono::microseconds((int)(1e4 - micro)));
        }
    }
}

int main(int argc, char **argv) {

    std::thread listener(listener_thread);
    std::thread setPose(settingPose_thread);
    std::thread command(command_thread);

    listener.join();
    setPose.join();
    command.join();

    return 0;

}