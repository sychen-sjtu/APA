#include "common/math_utils.h"

namespace APA{
    double normalize_angle(double angle){
        while(angle > M_PI){
            angle -= 2 * M_PI;
        }
        while(angle < -M_PI){
            angle += 2 * M_PI;
        }
        return angle;
    }
}

