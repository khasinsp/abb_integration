#include <bits/stdc++.h>

void test(std::vector<float> vec) {
    for (float v : vec) {
        std::cout << v << ", ";
    }
    std::cout << std::endl;
}

std::vector<float> current_joints;
#define FREQ 1000

int main() {
    // float x[6] = {0, 1, 10, 232, 2321, 123};
    // current_joints.assign(x, x+6);
    // for (float c : current_joints) {
    //     std::cout << c << ", ";
    // }
    // std::cout << std::endl;

    std::vector<int> a = {1, 2, 3, 4, 5, 6};

    int idx = 0;

    for (int k = 0; k < idx; k++) {
        a.pop_back();
    }

    for (int s : a) {
        std::cout << s << std::endl;
    }
}