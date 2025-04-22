#include <bits/stdc++.h>
#include "../include/tcp_server.h"

#define LOCAL_HOST "192.168.0.60"
#define LOCAL_PORT 6008

#define numJoints 6

#define ANGLE_THRESHOLD 360
#define SPEED_THRESHOLD 360

#define FREQ 40

struct RobotState {
    float values[numJoints];
};

RobotState unpack(const std::string& buffer) {
    RobotState state;

    for (int i = 0; i < numJoints; ++i) {
        std::memcpy(&state.values[i], buffer.data() + i * sizeof(float), sizeof(float));
    }

    return state;
}

bool check_buffer(std::string &buffer) {
    if (buffer.size() < 2) return false;

    return static_cast<unsigned char>(buffer[buffer.size() - 2]) == 0xAA &&
           static_cast<unsigned char>(buffer[buffer.size() - 1]) == 0x55;
}

std::queue<std::vector<float>> command_arr;
std::vector<float> current_joints;

void generate_motion(std::queue<std::vector<float>> &command_arr, std::vector<float> command) {

    // calculate length of command array given frequency and speed
    float speed = command.back();
    command.pop_back();
    // for (int i = 0; i < command.size(); i++) {
    //     command[i] += current_joints[i];
    // }
    float max_angle = *std::max_element(command.begin(), command.end(), [](float a, float b) {return std::abs(a) < std::abs(b);});
    max_angle = std::abs(max_angle);

    float t_end = max_angle / speed;
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

std::vector<int8_t> generate_command(std::vector<float> command, bool last_command) {
    std::vector<int8_t> buffer;
    const uint8_t end_of_command[] = { 0xAA, 0x55 };
    int32_t last_value = last_command ? -1 : 5;
    int8_t last_value_bytes [sizeof(int32_t)];
    std::memcpy(last_value_bytes, &last_value, sizeof(int32_t));

    float s = 0.0;

    // Copy Joint values to send buffer
    for (float joint : command) {
        int8_t bytes[sizeof(float)];
        std::memcpy(bytes, &joint, sizeof(float));
        buffer.insert(buffer.end(), bytes, bytes + sizeof(float));
    }

    // Copy Start time to send buffer
    int8_t timeBytes[sizeof(float)];
    std::memcpy(timeBytes, &s, sizeof(float));
    buffer.insert(buffer.end(), timeBytes, timeBytes + sizeof(float));

    // Copy last value bytes (10) to send buffer
    buffer.insert(buffer.end(), last_value_bytes, last_value_bytes + sizeof(int32_t));
    
    // Copy End of Command bytes (0xAA, 0x55) to send buffer
    buffer.insert(buffer.end(), std::begin(end_of_command), std::end(end_of_command));

    return buffer;
}

int main_thread() {
    TCPServer *socketServer = new TCPServer(LOCAL_HOST, LOCAL_PORT);

    socketServer->start_();
    socketServer->setToNonblockingMode_();

    std::string in_buffer;
    std::string received_main_buffer;

    std::vector<int8_t> out_buffer;

    RobotState current_state;

    bool commanded = false;

    auto start = std::chrono::high_resolution_clock::now();

    while (true) {
        ssize_t received_bytes = socketServer->recv_(in_buffer);
        if (received_bytes > 0) {
            received_main_buffer += in_buffer;
        }

        if (check_buffer(received_main_buffer)) {
            received_main_buffer.pop_back();
            received_main_buffer.pop_back();
            current_state = unpack(received_main_buffer);
            current_joints.assign(current_state.values, current_state.values + numJoints);
            received_main_buffer = "";

            if (command_arr.size() > 1) {
                // commanded = true;
                std::vector<float> command = command_arr.front();
                command_arr.pop();
                out_buffer = generate_command(command, false);

                std::cout << "state:" << std::endl;
                for (float c : current_joints) {
                    std::cout << std::fixed << std::setprecision(3) << c << ", ";
                }
                std::cout << std::endl;
                
                std::cout << "command:" << std::endl;
                for (float c : command) {
                    std::cout << std::fixed << std::setprecision(3) << c << ", ";
                }
                std::cout << std::endl;
                
                ssize_t sent_bytes = socketServer->send_(out_buffer);
            }
            else if (command_arr.size() == 1) {
                commanded = true;
                std::vector<float> command = command_arr.front();
                command_arr.pop();
                out_buffer = generate_command(command, true);
                // int res;
                // std::memcpy(&res, &out_buffer[out_buffer.size() - 6], sizeof(int32_t));
                // std::cout << res << std::endl;
                ssize_t sent_bytes = socketServer->send_(out_buffer);
            }
            else {
                // if (commanded) {
                //     std::cout << "state:" << std::endl;
                //     for (float c : current_joints) {
                //         std::cout << c << ", ";
                //     }
                //     std::cout << std::endl;
                // }
                out_buffer = generate_command(current_joints, true);
                ssize_t sent_bytes = socketServer->send_(out_buffer);
                // if (command_arr.size() == 1) {
                //     std::vector<float> command = command_arr.front();
                //     out_buffer = generate_command(command, true);
                //     ssize_t sent_bytes = socketServer->send_(out_buffer);
                // }
                // else {
                //     out_buffer = generate_command(current_joints, true);
                //     ssize_t sent_bytes = socketServer->send_(out_buffer);
                // }
                
            }
            // if (command_arr.size() > 0) std::cout << command_arr.size() << std::endl;

            auto end = std::chrono::high_resolution_clock::now();
            int loop_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            // std::cout << std::fixed << std::setprecision(3) << loop_time / 1000.0 << std::endl;
            start = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::microseconds(1 / FREQ - loop_time));
        }
    }
}

int main() {
    std::thread main_t(main_thread);
    std::thread user_input_t(user_input_thread);

    main_t.join();
    user_input_t.join();

    return 0;
}