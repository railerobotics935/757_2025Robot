
#include "Constants.h"

namespace CameraConstants {

  double GetStandardDeviationFromDistance(double distance) {
    if(distance < 1.0)
      return kMinStandardDeviation;
    else if (distance > 10.0)
      return kMaxStandardDeviation;
    else
      return ((distance - 1.0) / 9.0) * (kMaxStandardDeviation - kMinStandardDeviation) + kMinStandardDeviation;
  }
}




