#include <vector>
#include <string>
#include <random>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "MassBody.h"

MassBody::MassBody(std::string inname, double inmass, long double inrad, std::vector<long double> pos, std::vector<long double> vel)
{
    this->name = inname;
    this->mass = inmass;
    this->rad = inrad;
    this->vel = vel;
    this->pos = pos;
    this->acc = {0.0, 0.0, 0.0};

    std::random_device rd;
    std::default_random_engine rng(rd());
    std::uniform_int_distribution <> rng_col(0, 255);

    this->colour = cv::Scalar(rng_col(rng), rng_col(rng), rng_col(rng));
}

void MassBody::printParam()
{
    std::cout << name << ": \n" << "m: " << mass << "\tpos: ";

    for (std::vector<long double>::iterator it = pos.begin(); it != pos.end(); ++it)
        std::cout << *it << ", ";

    std::cout << "\tvel: ";

    for (std::vector<long double>::iterator it = vel.begin(); it != vel.end(); ++it)
        std::cout << *it << ", ";

    std::cout << "\tacc: ";

    for (std::vector<long double>::iterator it = oldacc.begin(); it != oldacc.end(); ++it)
        std::cout << *it << ", ";

    std::cout << std::endl;
}

std::vector<long double> MassBody::accUpdate(MassBody other)
{
    std::vector<long double> delta_d;
    delta_d.push_back(other.pos.at(0) - this->pos.at(0));
    delta_d.push_back(other.pos.at(1) - this->pos.at(1));
    delta_d.push_back(other.pos.at(2) - this->pos.at(2));

    double dist_squa = pow(delta_d.at(0), 2) + pow(delta_d.at(1), 2) + pow(delta_d.at(2), 2);
    double dist = sqrt(dist_squa);

    if (dist_squa == 0.0)
        // join bodies?
        std::exit(EXIT_FAILURE);

    double force_total = GRAV_CONST * (this->mass * other.mass) / dist_squa;
    double force_x = force_total * (delta_d.at(0) / dist);
    double force_y = force_total * (delta_d.at(1) / dist);
    double force_z = force_total * (delta_d.at(2) / dist);

    acc.at(0) += force_x / mass;
    acc.at(1) += force_y / mass;
    acc.at(2) += force_z / mass;

    return delta_d;
}

void MassBody::posUpdate(double timestep)
{

    // add position to history
    past.push_back(pos);

    // update position using old velocity and acceleration
    pos.at(0) += vel.at(0) * timestep + (0.5 * acc.at(0) * pow(timestep, 2));
    pos.at(1) += vel.at(1) * timestep + (0.5 * acc.at(1) * pow(timestep, 2));
    pos.at(2) += vel.at(2) * timestep + (0.5 * acc.at(2) * pow(timestep, 2));

    // update velocity
    vel.at(0) += acc.at(0) * timestep;
    vel.at(1) += acc.at(1) * timestep;
    vel.at(2) += acc.at(2) * timestep;

    oldacc = acc;

    acc.at(0) = 0.0;
    acc.at(1) = 0.0;
    acc.at(2) = 0.0;
}

void MassBody::addToCanvas(canvasPasser passed, bool drawpast)
{
    cv::Mat canvas = passed.canvas;

    const double maxdist = passed.maxdist;
    const double maxmass = passed.maxmass;
    const double maxrad = passed.maxrad;

    const double start_maxdist = passed.start_maxdist;

    const int canHeight = passed.canHeight;
    const int canWidth = passed.canWidth;

    const float scaling = 0.5;
    // convert the space coordinates to coordinates on the canvas
    cv::Point centre = cv::Point((int)(std::min(canHeight, canWidth) * scaling * (pos.at(0) / maxdist) + 0.5 * canWidth),
                                 (int)(std::min(canHeight, canWidth) * scaling * (pos.at(1) / maxdist) + 0.5 * canHeight));

    // get the radius of the circle to display, sized relative to scale
    int cirrad = (int)(0.5 + (rad / maxrad * 50) * (start_maxdist / maxdist));

    cv::circle(canvas, centre, cirrad, this->colour, cv::FILLED, cv::LINE_8);

    if (!drawpast)
        return;

    // get bodies past coordinates, convert them to canvas coordinates with the current scaling
    std::vector<cv::Point> paspoints;

    for (std::vector<std::vector<long double>>::iterator it = past.begin(); it != past.end(); ++it)
    {
        paspoints.push_back(cv::Point((int)(std::min(canHeight, canWidth) * scaling * (it->at(0) / maxdist) + 0.5 * canWidth),
                                      (int)(std::min(canHeight, canWidth) * scaling * (it->at(1) / maxdist) + 0.5 * canHeight)));
    }

    paspoints.push_back(cv::Point((int)(std::min(canHeight, canWidth) * scaling * (pos.at(0) / maxdist) + 0.5 * canWidth),
                                  (int)(std::min(canHeight, canWidth) * scaling * (pos.at(1) / maxdist) + 0.5 * canHeight)));

    cv::polylines(canvas, paspoints, false, colour * 0.4, 1, cv::LINE_8);
}
