#include <bits/stdc++.h>

void test(std::vector<float> vec) {
    for (float v : vec) {
        std::cout << v << ", ";
    }
    std::cout << std::endl;
}

std::vector<float> current_joints;

int main() {
    float x[6] = {0, 1, 10, 232, 2321, 123};
    current_joints.assign(x, x+6);
    for (float c : current_joints) {
        std::cout << c << ", ";
    }
    std::cout << std::endl;
}