#include "MicroSafeController.h"
#include <iostream>

int main() {
    MicroSafeController safe;

    float inputs[] = {0.2f, 0.5f, 2.0f, -3.0f, 0.1f};

    for (float x : inputs) {
        float out = safe.apply(x, x);
        std::cout << "in=" << x << " out=" << out << std::endl;
    }

    return 0;
}