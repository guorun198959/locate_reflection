//
// Created by waxz on 18-7-20.
//

#ifndef LOCATE_REFLECTION_TYPES_H
#define LOCATE_REFLECTION_TYPES_H
namespace tupe_util {
    struct Point2d {
        double x;
        double y;

        Point2d() {
            x = 0.0;
            y = 0.0;
        }

        Point2d(double x1, double y1) {
            x = x1;
            y = y1;
        }
    };

    struct Point2f {
        float x;
        float y;

        Point2f() {
            x = 0.0;
            y = 0.0;
        }

        Point2f(float x1, float y1) {
            x = x1;
            y = y1;
        }
    };
}

#endif //LOCATE_REFLECTION_TYPES_H
