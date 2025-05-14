// #define NOLOADED_RUN
#define LOADED_RUN
#include "./include/navigation/navigation.hpp"

#include <string>

int main(int argc, char *argv[]) {
    double kp_omega = std::stod(argv[1]);

    Navigation naviation(0.1155, 0.245, 0.0273, 640, 480, 30, false, 1.0, 0.0001);
    naviation.followLine();
    return 0;
}