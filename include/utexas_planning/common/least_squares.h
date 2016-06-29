#ifndef UTEXAS_PLANNING_LEAST_SQUARES_H
#define UTEXAS_PLANNING_LEAST_SQUARES_H

#include <math.h>
#include <vector>

namespace utexas_planning {

  /**
   * Eqns (9) and (10) from http://mathworld.wolfram.com/LeastSquaresFittingExponential.html
   */
  void leastSquaresExponentialFit2(const std::vector<float> &x,
                                   const std::vector<float> &y,
                                   float &a,
                                   float &b) {
    float x2ysum = 0.0f;
    float ylnysum = 0.0f;
    float xysum = 0.0f;
    float xylnysum = 0.0f;
    float ysum = 0.0f;

    for (unsigned int i = 0; i < x.size(); ++i) {
      float lny = logf(y[i]);
      float xy = x[i] * y[i];
      x2ysum += x[i] * xy;
      ylnysum += y[i] * lny;
      xysum += xy;
      xylnysum += xy * lny;
      ysum += y[i];
    }

    a = ((x2ysum * ylnysum) - (xysum * xylnysum)) / ((ysum * x2ysum) - (xysum * xysum));
    a = expf(a);
    b = ((ysum * xylnysum) - (xysum * ylnysum)) / ((ysum * x2ysum) - (xysum * xysum));
  }

};

#endif /* end of include guard: UTEXAS_PLANNING_LEAST_SQUARES_H */
