#ifndef VISION_HPP
#define VISION_HPP

#include "../include/image/image.hpp"

#include <vector>
#include <stdio.h>

#define N_SLICES 4

class Vision{
private:
    std::vector<Image> slices;

public:
    Vision() : slices(N_SLICES)
    {}

    void getOutputVision() 
};


#endif // VISION_HPP