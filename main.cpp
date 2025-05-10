#define NOLOADED_RUN
#include "./include/navigation/navigation.hpp"

int main() {
    Navigation naviation(0.1155, 0.1155, 0.026, 0.0215, 1.0, 2.0, 2.0, );
    naviation.navigate();
}