#include "aerotwin/common/Version.hpp"

#include <iostream>

int main() {
    std::cout << "AeroTwin version: " << aerotwin::version() << '\n';
    return 0;
}
