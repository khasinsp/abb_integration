#include <bits/stdc++.h>

void test(std::vector<float> vec) {
    for (float v : vec) {
        std::cout << v << ", ";
    }
    std::cout << std::endl;
}

int main() {
    float x[6] = {0, 1, 10, 232, 2321, 123};
    std::vector<float> vec(std::begin(x), std::end(x));
    test(vec);
    return 0;
}