#ifndef MassBody_H
#define MassBody_H

#include <vector>
#include <string>

#include <opencv2/core.hpp>

const double GRAV_CONST = 6.67408e-11;

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
        void addToCanvas(cv::Mat canvas, double maxdist, double maxmass, double maxrad, bool drawpast=true);
};
#endif
