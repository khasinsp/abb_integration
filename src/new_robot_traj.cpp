#include <bits/stdc++.h>
#include <sys/syscall.h>
#include <linux/sched.h>
#include "../include/tcp_server.h"

#define LOCAL_HOST "192.168.0.60"
#define LOCAL_PORT 6008

#define numJoints 6 //Robot has 6 joints

#define ANGLE_THRESHOLD 360 //Threshold the command angles
#define SPEED_THRESHOLD 360 //Threshold the speed

#define FREQ 20.0

/*
Set FIFO Policy with priority for the current Thread
*/
void set_realtime_priority(int priority) {
    struct sched_param param;
    param.sched_priority = priority;

    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        perror("Unable to set realtime priority");
        exit(EXIT_FAILURE);
    }
}

/*
Set CPU Affinity for the current Thread
*/
void set_CPU(size_t cpu)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);

    CPU_SET(cpu, &mask);

    // pid = 0 means "calling process"
    if (sched_setaffinity(0, sizeof(mask), &mask) == -1)
    {
        std::cerr << "Error setting CPU affinity: "
                  << std::strerror(errno) << std::endl;
    }
}

/*
SIGXCPU Handler
*/
void sigxcpu_handler(int signum) {
    std::cerr << "Runtime overrun detected: Task exceeded allocated runtime" << std::endl;
}
/*
Signal Handler for handling runtime overruns
*/
void setup_signal_handler() {
    struct sigaction sa;
    sa.sa_handler = sigxcpu_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGXCPU, &sa, NULL) != 0) {
        perror("Failed to set SIGXCPU handler");
        exit(EXIT_FAILURE);
    }
}

/*
Function for setting Deadline Policy to current Thread
*/
struct sched_attr {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};
void set_realtime_deadline(unsigned long runtime, unsigned long deadline, unsigned long period)
{
    struct sched_attr attr;
    int ret;

    // Zero out the structure
    memset(&attr, 0, sizeof(attr));

    // Set the scheduling policy to SCHED_DEADLINE
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = runtime;
    attr.sched_deadline = deadline;
    attr.sched_period = period;

    // Enable Overrun detection
    attr.sched_flags |= SCHED_FLAG_DL_OVERRUN;

    // Use syscall to set the scheduling policy and parameters
    ret = syscall(SYS_sched_setattr, gettid(), &attr, 0);
    if (ret != 0)
    {
        perror("Failed to set SCHED_DEADLINE");
        exit(EXIT_FAILURE);
    }
}

/*
RobotState for unpacking received bytes and copying them to readable joint states
*/
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

/*
check buffer for checking the received message
*/
bool check_buffer(std::string &buffer) {
    if (buffer.size() < 2) return false;

    return static_cast<unsigned char>(buffer[buffer.size() - 2]) == 0xAA &&
           static_cast<unsigned char>(buffer[buffer.size() - 1]) == 0x55;
}

std::queue<std::vector<float>> command_arr;
std::vector<float> current_joints;

/*
Sets the interpolated commands into the command_arr queue
Start Position is current joint positions
End Position is absolute command
*/
void generate_motion(std::queue<std::vector<float>> &command_arr, std::vector<float> command) {

    // calculate length of command array given frequency and speed
    float speed = command.back();
    command.pop_back();

    std::vector<float> current_joints_copy = current_joints;
    std::vector<float> max_vec;
    for (int i = 0; i < command.size(); i++) {
        max_vec.push_back(command[i] - current_joints_copy[i]);
    }
    float max_angle = *std::max_element(max_vec.begin(), max_vec.end(), [](float a, float b) {return std::abs(a) < std::abs(b);});
    max_angle = std::abs(max_angle);

    float t_end = max_angle / speed;
    std::cout << "Max Angle is: " << max_angle << std::endl;
    int commandlen = max_angle / speed * FREQ;

    for (int i = 0; i < commandlen; i++) {
        std::vector<float> c(command.size());
        for (int j = 0; j < c.size(); j++) {
            c[j] = current_joints_copy[j] + (command[j] - current_joints_copy[j]) / (commandlen - 1) * i;
            
        }
        command_arr.push(c);
    }

    return;

}

/*
Thread for setting commands
*/
void user_input_thread() {
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // std::vector<float> command = {0, 0, 0, 0, 0, 0, 90};
    // generate_motion(command_arr, command);

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

/*
Generates a command readable for the robot
*/
std::vector<int8_t> generate_command(std::vector<float> command, bool last_command) {
    std::vector<int8_t> buffer;
    const uint8_t end_of_command[] = { 0xAA, 0x55 };
    int32_t last_value = last_command ? -1 : 10;
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

/*
Main communication Thread
*/
int main_thread() {

    set_CPU(0);

    TCPServer *socketServer = new TCPServer(LOCAL_HOST, LOCAL_PORT);

    socketServer->start_();
    socketServer->setToNonblockingMode_();

    std::string in_buffer;
    std::string received_main_buffer;

    std::vector<int8_t> out_buffer;

    RobotState current_state;

    std::vector<float> command;

    float period = 1 / FREQ * 1.0e6;

    set_realtime_priority(99);

    auto start = std::chrono::high_resolution_clock::now();
    short i = 0;
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
            if (i == 0) {
                command = current_joints;
                i = 1;
            }

            if (command_arr.size() > 0) {
                command = command_arr.front();
                if (command_arr.size() > 1) command_arr.pop();
                out_buffer = generate_command(command, false);
                ssize_t sent_bytes = socketServer->send_(out_buffer);
            }
            else {
                out_buffer = generate_command(command, false);
                ssize_t sent_bytes = socketServer->send_(out_buffer);
            }

            auto end = std::chrono::high_resolution_clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            std::this_thread::sleep_for(std::chrono::microseconds((int)period - dt));
            // end = std::chrono::high_resolution_clock::now();
            // auto loop_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            // std::cout << loop_time / 1000.0 << std::endl;

            start = std::chrono::high_resolution_clock::now();
            if (dt > 1.0 / FREQ * 1.0e6) {
                int idx = dt * FREQ / 1.0e6  - 1;
                for (int k = 0; k < idx; k++) {
                    if (command_arr.size() > 1) {
                        command_arr.pop();
                        std::vector<float> new_command(numJoints);
                        float t = 1 / FREQ - dt;
                        std::queue<std::vector<float>> command_arr_backup = command_arr;
                        std::vector<float> first_command = command_arr_backup.front();
                        command_arr_backup.pop();
                        std::vector<float> second_command = command_arr_backup.front();
                        for (int k = 0; k < numJoints; k++) {
                            if (command_arr.size() >= 2) {
                                new_command[k] = (first_command[k] * t + second_command[k] * (1.0 / FREQ - t)) * FREQ;
                            }
                        }
                        command_arr.front() = new_command;
                    }
                }
            }
        }
    }
}

/*
Main Function
*/
int main() {
    std::thread main_t(main_thread);
    std::thread user_input_t(user_input_thread);

    main_t.join();
    user_input_t.join();

    return 0;
}