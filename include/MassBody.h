#ifndef MassBody_H
#define MassBody_H

#include <vector>
#include <string>

#include <opencv2/core.hpp>

const double GRAV_CONST = 6.67408e-11;

struct canvasPasser
{
    cv::Mat canvas;
    const double maxdist, maxmass, maxrad;
    const double start_maxdist;
    const int canHeight, canWidth;

};

class MassBody
{
    public:
        std::string name;
        double mass;
        double rad;
        cv::Scalar colour;
        std::vector<double> acc;
        std::vector<double> vel;
        std::vector<double> pos;
        std::vector<double> oldacc;
        std::vector<std::vector<double>> past;

        MassBody(std::string inname, double inmass, double inrad, std::vector<double> pos, std::vector<double> vel);

        void printParam();
        std::vector<double> accUpdate(MassBody other);
        void posUpdate(double timestep);
        void addToCanvas(canvasPasser passed, bool drawpast = true);
};
#endif
