/**
 * @file utils.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <array>


struct TemperatureCurve {
    float targetTemperature; // Target temperature in centi Celsius
    float endTemperature;   // Ending temperature in centi Celsius
    uint32_t rampUpDuration;  // Duration in milliseconds to reach the end temperature
    uint32_t holdDuration;    // Duration in milliseconds to hold the end temperature
    uint32_t coolDownDuration; // Duration in milliseconds to cool down to the start temperature
};

template <typename T, size_t Size>
class CircularFifo {
public:
    CircularFifo() = default;
    ~CircularFifo() = default;

    void push(T value) {
        if (count < Size) {
            buffer[tail] = value;
            tail = (tail + 1) % Size;
            count++;
        } else {
            buffer[tail] = value;
            tail = (tail + 1) % Size;            
            head = (head + 1) % Size; 
        }
    }

    T pop() {
        T value = buffer[head];
        head = (head + 1) % Size;
        count--;
        return value;
    }

    T get() {
        return buffer[head];
    }

    T sum() const {
        T sum = 0;
        for (size_t i = 0; i < count; ++i) {
            sum += buffer[i];
        }
        return sum;
    }

    bool isEmpty() const {
        return count == 0;
    }
    bool isFull() const {
        return count == Size;
    }
private:
    std::array<T, Size> buffer;
    size_t head = 0;
    size_t tail = 0;
    size_t count = 0;
};

#endif // UTILS_H