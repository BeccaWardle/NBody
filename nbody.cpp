#include <iostream>
#include <string>
#include <cmath>
#include <vector>

//random
#include <random>
#include <time.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "include/MassBody.h"

using namespace cv;
using namespace std;

const int WAITTIME = 20;
const int canHeight = 1300;
const int canWidth = 1300;

double start_maxMass;
double start_maxRad;
double start_maxDist;


int main(int argc, char *argv[])
{
    //int opt;

    /*while ((opt = getopt(argc, argv, "h:w:ano:i:")) != EOF)
    {
        switch(opt)
        {
        case 'a':
        average = true;
        break;
        case 'r':
        white = true;
        break;
        case 'h':
        blockH = std::stoi(optarg, NULL, 10);
        break;
        case 'w':
        blockW = std::stoi(optarg, NULL, 10);
        break;
        case 'o':
        outName = (String) optarg;
        break;
        case 'i':
        inName = (String) optarg;
        break;
        case '?':
        if (optopt == 'w' || optopt == 'w' || optopt == 'i' || optopt == 'o')
        fprintf(stderr, "-%c requires an argument\n", optopt);
        else if (isprint(optopt))
        fprintf(stderr, "Unknown option -%c", optopt);
        else
        fprintf(stderr, "Unknown option character %x\n", optopt);
        default:
        exit(EXIT_FAILURE);

        }
    //}*/


    // add all bodies to the simulation with values
    vector<MassBody> bodies;
    MassBody Earth = MassBody("Eath", 5.972e24, 6371, vector<double> {0, 0, 0}, vector<double> {0, 0, 0});
    MassBody MoonOne = MassBody("Moon1", 7.348e22, 1737, vector<double> {0, -200e6, 0}, vector<double> {-150.0, 0, 0});
    MassBody MoonTwo = MassBody("Moon2", 7.348e22, 1737, vector<double> {0, 200e6, 0}, vector<double> {150.0, 0, 0});
    MassBody MoonThree = MassBody("Moon3", 7.348e22, 1737, vector<double> {200e6, 0, 0}, vector<double> {0.0, -150, 0});
    MassBody MoonFour = MassBody("Moon4", 7.348e22, 1737, vector<double> {-200e6, 0, 0}, vector<double> {0.0, 150, 0});
    // bodies.push_back(Earth);
    bodies.push_back(MoonOne);
    bodies.push_back(MoonTwo);
    bodies.push_back(MoonThree);
    bodies.push_back(MoonFour);

    double timestep = 108000;

    double maxMass = 0.0;
    double maxRad = 0.0;
    // not sure where this value comes from
    double maxDist = 600e6;
    start_maxDist= maxDist;

    Mat canvas = Mat::zeros(canHeight, canWidth, CV_8UC3);
    char canName[] = "frame";
    namedWindow(canName);

    moveWindow(canName, 50, 50);
    // get the inital values of all bodies and plot them before first step
    // TODO: get maxDist too?
    for (vector<MassBody>::iterator it = bodies.begin(); it != bodies.end(); ++it) {
        if (it->mass > maxMass)
            start_maxMass = maxMass = it->mass;

        if (it->rad > maxRad)
            start_maxRad = maxRad = it->rad;

        canvasPasser passer {canvas, maxDist, maxMass, maxRad, start_maxDist, canHeight, canWidth};
        it->addToCanvas(passer);
        it->printParam();
    }

    imshow(canName, canvas);
    waitKey(WAITTIME);

    cout << "maxMass: " << maxMass << "\nmaxRad: " << maxRad << endl;
    while (1)
    {
        system("clear");
        canvas = Mat::zeros(canHeight, canWidth, CV_8UC3);
        moveWindow(canName, 50, 50);

        for (vector<MassBody>::iterator lowIt = bodies.begin(); lowIt != bodies.end(); ++lowIt)
        {
            for (vector<MassBody>::iterator upIt = bodies.begin(); upIt != bodies.end(); ++upIt)
            {
                if (lowIt->name == upIt->name)
                    continue;

                // TODO: use lambda?
                vector<double> currDist = lowIt->accUpdate(*upIt);
                for (vector<double>::iterator it = currDist.begin(); it != currDist.end(); ++it)
                {
                    if (fabs(*it) > maxDist)
                    {
                        maxDist = fabs(*it);
                        cout << "New maxdist: " << maxDist << endl;
                    }
                }

            }
        }

        for (vector<MassBody>::iterator it = bodies.begin(); it != bodies.end(); ++it)
        {
            it->posUpdate(timestep);
            canvasPasser passer {canvas, maxDist, maxMass, maxRad, start_maxDist, canHeight, canWidth};

            it->addToCanvas(passer);
            it->printParam();
        }
        imshow(canName, canvas);
        //waitKey(50);
        waitKey(WAITTIME);
    }

    destroyAllWindows();
    return 0;
}
