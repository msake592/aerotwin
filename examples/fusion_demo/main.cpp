#include "FusionReport.hpp"

#include <exception>
#include <iostream>

#if defined(__unix__) || defined(__APPLE__)
#include <signal.h>
#endif

int main(int argc, char**) {
    if (argc != 1) {
        std::cerr << "Usage: aerotwin_fusion_demo\n";
        return 1;
    }
#if defined(__unix__) || defined(__APPLE__)
    // This standalone process reports EPIPE through the stdout stream state.
    struct sigaction action {};
    action.sa_handler = SIG_IGN;
    if (sigemptyset(&action.sa_mask) != 0 || sigaction(SIGPIPE, &action, nullptr) != 0) {
        std::cerr << "Fusion demo failed: could not configure SIGPIPE handling\n";
        return 1;
    }
#endif
    try {
        return aerotwin::fusion_demo::writeFusionReport(aerotwin::fusion_demo::acquireScenario(),
                                                       std::cout, std::cerr);
    } catch (const std::exception& error) {
        std::cerr << "Fusion demo failed: " << error.what() << '\n';
        return 1;
    }
}
