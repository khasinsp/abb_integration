#include <bits/stdc++.h>
#include <sys/syscall.h>
#include <linux/sched.h>
#include "../include/tcp_server.h"

#define LOCAL_HOST "192.168.0.60"
#define TX_PORT 6009
#define RX_PORT 6008

#define numJoints 6 //Robot has 6 joints

#define ANGLE_THRESHOLD 360 //Threshold the command angles
#define SPEED_THRESHOLD 1000 //Threshold the speed

#define FREQ 80.0

#define ZONE 1000

/*
Set FIFO Policy with priority for the current Thread
*/
void set_realtime_priority(int priority) {
    struct sched_param param;
    param.sched_priority = priority;

    if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
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

bool check_received_state(std::string &buffer) {
    if (buffer.size() < 2) return false;

    return static_cast<unsigned char>(buffer[buffer.size() - 2]) == 0xAA &&
           static_cast<unsigned char>(buffer[buffer.size() - 1]) == 0x55;
}   

/*
check buffer for checking the received message
*/
bool check_received_message(const std::string &buffer) {
    return buffer.size() == 4 &&
           static_cast<unsigned char>(buffer[0]) == 0x6E &&
           static_cast<unsigned char>(buffer[1]) == 0x65 &&
           static_cast<unsigned char>(buffer[2]) == 0x78 &&
           static_cast<unsigned char>(buffer[3]) == 0x74;
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

    std::vector<float> current_joints_copy;

    if (command_arr.size() > 0) {
        current_joints_copy = command_arr.front();
    }
    else  {
        current_joints_copy = current_joints;
    }
    std::vector<float> max_vec;
    for (int i = 0; i < command.size(); i++) {
        max_vec.push_back(command[i] - current_joints_copy[i]);
    }
    float max_angle = *std::max_element(max_vec.begin(), max_vec.end(), [](float a, float b) {return std::abs(a) < std::abs(b);});
    max_angle = std::abs(max_angle);

    float t_end = max_angle / speed;
    // std::cout << "Max Angle is: " << max_angle << std::endl;
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
Generates a command readable for the robot
*/
std::vector<int8_t> generate_command(std::vector<float> command, bool last_command) {
    std::vector<int8_t> buffer;
    const uint8_t end_of_command[] = { 0xAA, 0x55 };
    int32_t last_value = last_command ? -1 : ZONE;
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

void write_to_queue(std::queue<std::pair<std::vector<float>, unsigned long>> &vec, std::pair<std::vector<float>, unsigned long> pair) {
    vec.push(pair);
}

void csv_thread(std::ofstream *csv_ptr, std::queue<std::pair<std::vector<float>, unsigned long>> *queue_ptr) {
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // set_CPU(1);
    // set_realtime_priority(99);

    while (true) {
        if (!(*queue_ptr).empty()) {
            auto data = (*queue_ptr).front();
            (*queue_ptr).pop();
            auto vec = data.first;
            auto ts = data.second;
            (*csv_ptr) << ts << ",";
            for (int i = 0; i < vec.size(); i++) {
                (*csv_ptr) << vec[i];
                if (i < vec.size() - 1) (*csv_ptr) << ",";
            }
            (*csv_ptr) << "\n";
            (*csv_ptr).flush();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds((int) (1.0 / FREQ * 1.0e3)));
    }

    std::cout << "successfully wrote csv file" << std::endl;
    exit(EXIT_SUCCESS);
}

void user_input(float move, float v) {

    float move_time = 2.0 * move / v;

    std::vector<float> command(7);

    command = {move, move, move, move, move, move, v};
    generate_motion(command_arr, command);

}

std::queue<std::pair<std::vector<float>, unsigned long>> act_q;
std::queue<std::pair<std::vector<float>, unsigned long>> com_q;

void receive_thread() {
    set_CPU(0);
    set_realtime_priority(99);

    TCPServer *socketServerRX = new TCPServer(LOCAL_HOST, RX_PORT);
    socketServerRX->start_();
    socketServerRX->setToNonblockingMode_();

    std::string in_buffer;
    std::string received_main_buffer;

    std::vector<float> current_joints;

    RobotState current_bytes;

    std::pair<std::vector<float>, unsigned long> pos_act;
    while (true) {
        ssize_t received_bytes = socketServerRX->recv_(in_buffer);
        if (received_bytes > 0) {
            received_main_buffer += in_buffer;
        }

        if (check_received_state(received_main_buffer)) {
            unsigned long ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

            current_bytes = unpack(received_main_buffer);
            current_joints.assign(current_bytes.values, current_bytes.values + numJoints);

            received_main_buffer = "";

            pos_act.first = current_joints;
            pos_act.second = ts;

            write_to_queue(act_q, pos_act);
        }
        sched_yield();
    }
}

void send_thread() {
    set_CPU(0);
    set_realtime_priority(99);

    TCPServer *socketServerTX = new TCPServer(LOCAL_HOST, TX_PORT);
    socketServerTX->start_();
    socketServerTX->setToNonblockingMode_();

    std::string in_buffer;
    std::string received_main_buffer;

    RobotState current_bytes;

    std::vector<float> command;
    std::vector<int8_t> out_buffer;

    std::pair<std::vector<float>, unsigned long> pos_com;

    int counter = 0;

    float move;

    float speed = 50;

    bool first_state_received = false;

    while (true) {
        ssize_t received_bytes = socketServerTX->recv_(in_buffer);
        if (received_bytes > 0) {
            received_main_buffer += in_buffer;
        }

        if (!first_state_received) {
            if (check_received_state(received_main_buffer)) {
                first_state_received = true;
                current_bytes = unpack(received_main_buffer);
                current_joints.assign(current_bytes.values, current_bytes.values + numJoints);
                command_arr.push(current_joints);
                received_main_buffer = "";
            }

            if (command_arr.size() > 0) {
                command = command_arr.front();
                out_buffer = generate_command(command, false);
                socketServerTX->send_(out_buffer);
                if (command_arr.size() > 1) command_arr.pop();
            }
        }

        if (check_received_message(received_main_buffer)) {
            received_main_buffer = "";
            unsigned long ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

            if (command_arr.size() > 0) {
                command = command_arr.front();
                out_buffer = generate_command(command, false);
                socketServerTX->send_(out_buffer);
                if (command_arr.size() > 1) command_arr.pop();
            }

            pos_com.first = command;
            pos_com.second = ts;

            write_to_queue(com_q, pos_com);

            if (command_arr.size() == 1) {
                if (counter % 2 == 0) {
                    move = 10.0;
                }
                else {
                    move = -10.0;
                }
                user_input(move, speed);
                counter++;
            }
        }

        sched_yield();
    }
}

/*
Main Function
*/
int main() {
    std::ofstream act_csv("/home/urc/abb_integration/motion_precision/two_threads/02_05_4/act.csv");
    if (!act_csv.is_open()) {
        std::cerr << "Act CSV could not be opened" << std::endl;
    }
    act_csv << "timestamp," << "j1," << "j2," << "j3," << "j4," << "j5," << "j6\n";
    act_csv.flush();

    std::ofstream com_csv("/home/urc/abb_integration/motion_precision/two_threads/02_05_4/com.csv");
    if (!com_csv.is_open()) {
        std::cerr << "Com CSV could not be opened" << std::endl;
    }
    com_csv << "timestamp," << "j1," << "j2," << "j3," << "j4," << "j5," << "j6\n";
    com_csv.flush();

    std::thread recv_t(receive_thread);
    std::thread send_t(send_thread);
    std::thread act_csv_t(csv_thread, &act_csv, &act_q);
    std::thread com_csv_t(csv_thread, &com_csv, &com_q);

    recv_t.join();
    send_t.join();
    act_csv_t.join();
    com_csv_t.join();
}