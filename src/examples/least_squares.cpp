#include <iostream>

#include <utexas_planning/common/least_squares.h>

int main(int argc, const char *argv[]) {

  std::vector<float> x;
  x.push_back(0);
  x.push_back(1);
  x.push_back(2);
  x.push_back(3);

  std::vector<float> y;
  y.push_back(1.05);
  y.push_back(2.10);
  y.push_back(3.85);
  y.push_back(8.30);

  float a, b;
  utexas_planning::leastSquaresExponentialFit2(x, y, a, b);
  std::cout << "A: " << a << std::endl;
  std::cout << "B: " << b << std::endl;

  return 0;
}
