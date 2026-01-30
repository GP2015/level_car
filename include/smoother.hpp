#pragma once

#include <vector>

class Smoother {
    int size;
    std::vector<float> buffer;
    int index;
    int count;
    double sum;

   public:
    Smoother(int size);
    double add_value(double val);
    double get_average();
};