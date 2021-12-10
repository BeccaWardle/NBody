#ifndef MassBody_H
#define MassBody_H

#include <vector>
#include <string>

#include <opencv2/core.hpp>

const double GRAV_CONST = 6.67408e-11;

struct canvasPasser
{
    cv::Mat canvas;
    const long double maxdist, maxmass, maxrad;
    const long double start_maxdist;
    const int canHeight, canWidth;

};

class MassBody
{
    public:
        std::string name;
        double mass;
        double rad;
        cv::Scalar colour;
        std::vector<long double> acc;
        std::vector<long double> vel;
        std::vector<long double> pos;
        std::vector<long double> oldacc;
        std::vector<std::vector<long double>> past;

        MassBody(std::string inname, double inmass, long double inrad, std::vector<long double> pos, std::vector<long double> vel);

        void printParam();
        std::vector<long double> accUpdate(MassBody other);
        void posUpdate(double timestep);
        void addToCanvas(canvasPasser passed, bool drawpast = true);
};
#endif
