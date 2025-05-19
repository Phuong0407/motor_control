#ifndef VELOCITY_H
#define VELOCITY_H

#include <vector>
#include <cmath>

class Velocity {
public:
    Velocity();
    std::vector<int> calculateOffsetDiff(const std::vector<int>& offsets);
};

Velocity::Velocity() {}

std::vector<int> Velocity::calculateOffsetDiff(const std::vector<int>& offsets) {
    std::vector<int> offset_diff;
    for (size_t i = 1; i < offsets.size(); i++) {
        offset_diff.push_back(offsets[i] - offsets[i - 1]);
    }
    return offset_diff;
}

#endif // VELOCITY_H