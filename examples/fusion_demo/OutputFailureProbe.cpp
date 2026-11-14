// Test launcher that sets up the demo's stdout before exec.
#include <cstdio>
#include <string_view>

#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        return 2;
    }

    const std::string_view mode = argv[2];
    int output;
    if (mode == "closed-pipe") {
        int endpoints[2];
        if (pipe(endpoints) == -1) {
            std::perror("pipe");
            return 2;
        }
        close(endpoints[0]);
        output = endpoints[1];

        // The demo must handle SIGPIPE itself, even if CTest ignores it.
        struct sigaction action {};
        action.sa_handler = SIG_DFL;
        if (sigemptyset(&action.sa_mask) != 0 || sigaction(SIGPIPE, &action, nullptr) != 0) {
            std::perror("sigaction");
            return 2;
        }
        sigset_t signals;
        if (sigemptyset(&signals) != 0 || sigaddset(&signals, SIGPIPE) != 0 ||
            sigprocmask(SIG_UNBLOCK, &signals, nullptr) != 0) {
            std::perror("unblock SIGPIPE");
            return 2;
        }
    } else if (mode == "read-only") {
        output = open("/dev/null", O_RDONLY);
        if (output == -1) {
            std::perror("open /dev/null");
            return 2;
        }
    } else {
        return 2;
    }

    if (dup2(output, STDOUT_FILENO) == -1) {
        std::perror("dup2");
        return 2;
    }
    if (output != STDOUT_FILENO) {
        close(output);
    }
    execl(argv[1], argv[1], static_cast<char*>(nullptr));
    std::perror("exec fusion demo");
    return 2;
}
