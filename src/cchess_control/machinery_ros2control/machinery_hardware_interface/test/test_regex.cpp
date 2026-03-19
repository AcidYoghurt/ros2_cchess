#include <iostream>
#include <cstdio>
#include <regex>

int main() {
    const char* input = "JointAngleOffset: -10.000000(A1), -20.000000(A2), -30.000000(A3)";
    float a1 = 0.0f, a2 = 0.0f, a3 = 0.0f;

    // 使用 sscanf 提取括号前的浮点数
    if (sscanf(input, "JointAngleOffset: %f(A1), %f(A2), %f(A3)", &a1, &a2, &a3) == 3) {
        std::cout << "A1: " << a1 << ", A2: " << a2 << ", A3: " << a3 << std::endl;
    } else {
        std::cerr << "解析失败，未匹配到所有数字" << std::endl;
    }

    std::regex Correct_re("JointAngleOffset:\\s*.*");
    std::smatch match;
    std::cout << std::regex_search(input, Correct_re)<<std::endl;

    return 0;
}
