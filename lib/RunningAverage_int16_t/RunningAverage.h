//
//    FILE: RunningAverage.h
//  AUTHOR: Rob.Tillaart@gmail.com
// VERSION: 0.2.16
//    DATE: 2016-dec-01
// PURPOSE: RunningAverage library for Arduino
//     URL: https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningAverage
// HISTORY: See RunningAverage.cpp
//
// Released to the public domain
//
// backwards compatibility
// clr()   clear()
// add(x)  addValue(x)
// avg()   getAverage()

#ifndef RunningAverage_h
#define RunningAverage_h

#define RUNNINGAVERAGE_LIB_VERSION "0.2.16"

#include "Arduino.h"

class RunningAverage
{
public:
  RunningAverage(void);
  explicit RunningAverage(const int8_t);
  ~RunningAverage();

  void    clear();
  void    addValue(const int16_t);
  void    fillValue(const int16_t, const int8_t);
  int16_t   getValue(const int8_t);

  int16_t   getAverage();      // iterates over all elements.
  int16_t   getFastAverage() const;  // reuses previous values.

  // return statistical characteristics of the running average
  int16_t   getStandardDeviation() const;
  int16_t   getStandardError() const;

  // returns min/max added to the data-set since last clear
  int16_t   getMin() const { return _min; };
  int16_t   getMax() const { return _max; };

  // returns min/max from the values in the internal buffer
  int16_t   getMinInBuffer() const;
  int16_t   getMaxInBuffer() const;

  // return true if buffer is full
  bool    bufferIsFull() const { return _cnt == _size; };

  int16_t   getElement(int8_t idx) const;

  int8_t getSize() const { return _size; }
  int8_t getCount() const { return _cnt; }



protected:
  int8_t _size;
  int8_t _cnt;
  int8_t _idx;
  int32_t   _sum;
  int16_t*  _ar;
  int16_t   _min;
  int16_t   _max;
};

#endif
// END OF FILE