#pragma once
#include <stdio.h>

typedef enum
{
    NONE,
    KALMAN,
    AVRG,
    MEDIAN,
    EXPONENT_SMOOTH
} FilterType;

template <typename T>
T getAvrg(T value, uint16_t sample = 100)
{
    static uint16_t counter = 0;
    // const uint8_t measure = 20;
    static T preValue = 0;
    static T filteredValue = value;
    if (counter != sample)
    {
        counter++;
        preValue += value;
        return value;
    }
    else
    {
        counter = 0;
        filteredValue = preValue / sample;
    }
    return filteredValue;
}

template <typename T>
T getMedian(T value)
{
    static uint8_t counter = 0;
    const uint8_t measure = 3;
    static T val[3];

    counter != measure ? counter++ : counter=0; 

    val[counter] = value;
    return val[0] > val[1]
               ? (val[1] > val[2] ? val[1] : (val[0] < val[2] ? val[0] : val[2]))
               : (val[1] < val[2] ? val[1] : (val[0] < val[2] ? val[2] : val[0]));
}

template <typename T>
T getExpSmooth(T value, float koefficient)
{
    static T prevValue = value;
    prevValue = (1 - koefficient) * prevValue + koefficient * value;
    return prevValue;
}
