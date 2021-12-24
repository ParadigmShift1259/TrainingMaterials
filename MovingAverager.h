#ifndef _MovingAverager_h_
#define _MovingAverager_h_

class MovingAverager
{
public:
  MovingAverager(int filterLength);
  double addSample(double newValue);
  double getValue();
  void dumpFilter();

private:
  double *filterPointer;
  int filterLength;
  double lastValue;

  void initFilter();
  void shiftFilter(double nextValue);
  void computeAverage();
};

#endif