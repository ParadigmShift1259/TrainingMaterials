#include "MovingAverager.h"

MovingAverager::MovingAverager(int filterLength)
{
  this->filterLength = filterLength;
  this->filterPointer = new double[filterLength];
  this->lastValue = 0.0;
  initFilter();
}

double MovingAverager::addSample(double newValue)
{
  shiftFilter(newValue);
  computeAverage();
  return this->lastValue;
}

double MovingAverager::getValue()
{
  return this->lastValue;
}

void MovingAverager::dumpFilter()
{
  this->lastValue = 0.0;
  initFilter();
}

void MovingAverager::shiftFilter(double nextValue)
{
  for (int i = this->filterLength - 1; i > -1; i--)
  {
    if (i == 0)
    {
      *(filterPointer) = nextValue;
    }
    else
    {
      *(filterPointer + i) = *(filterPointer + (i - 1));
    }
  }
}

void MovingAverager::computeAverage()
{
  double sum = 0.0;
  for (int i = 0; i < this->filterLength; i++)
  {
    sum += *(filterPointer + i);
  }
  this->lastValue = sum / this->filterLength;
}

void MovingAverager::initFilter()
{
  for (int i = 0; i < this->filterLength; i++)
  {
    *(filterPointer + i) = 0.0;
  }
}