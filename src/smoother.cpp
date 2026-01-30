#include "smoother.hpp"

Smoother::Smoother(int size) : size(size),
                               buffer(size, 0.0),
                               index(0),
                               count(0),
                               sum(0.0) {}

double Smoother::add_value(double val) {
    sum -= buffer[index];

    buffer[index] = val;
    sum += val;

    index = (index + 1) % size;

    if (count < size)
        count++;

    return sum / count;
}

double Smoother::get_average() {
    return count > 0 ? sum / count : 0.0;
}
