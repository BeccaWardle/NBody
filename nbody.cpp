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
using namespace cv;
using namespace std;

const double grav_const = 6.67408e-11;
const int WAITTIME = 20;
const int canHeight = 1300;
const int canWidth = 1300;

double start_maxMass;
double start_maxRad;
double start_maxDist;


class Body
{
    public:
        string name;
        double mass;
        double rad;
        vector<double> acc;
        vector<double> vel;
        vector<double> pos;
        vector<double> oldacc;
        vector<vector<double>> past;

        Scalar colour;

        Body(string inName, double inMass, double inRad, vector<double> pos, vector<double> vel)
        {
            this->name = inName;
            this->mass = inMass;
            this->rad = inRad;
            this->vel = vel;
            this->pos = pos;
            this->acc = {0, 0, 0};

            random_device rd;
            default_random_engine rng(rd());
            uniform_int_distribution <> rng_col(0, 255);

            this->colour = Scalar(rng_col(rng), rng_col(rng), rng_col(rng));
        }

        void printParam()
        {
            cout << name << ": \n" << "m: " << mass << "\tpos: ";
            for (vector<double>::iterator it = pos.begin(); it != pos.end(); ++it)
            {
                cout << *it << ", ";
            }
            cout << "\tvel: ";
            for (vector<double>::iterator it = vel.begin(); it != vel.end(); ++it)
            {
                cout << *it << ", ";
            }
            cout << "\tacc: ";
            for (vector<double>::iterator it = oldacc.begin(); it != oldacc.end(); ++it)
            {
                cout << *it << ", ";
            }
            cout << endl;
        }

        vector<double> accAround(Body other)
        {
            vector<double> delta_d;
            delta_d.push_back(other.pos.at(0) - this->pos.at(0));
            delta_d.push_back(other.pos.at(1) - this->pos.at(1));
            delta_d.push_back(other.pos.at(2) - this->pos.at(2));

            double dist_squa = pow(delta_d.at(0), 2) + pow(delta_d.at(1), 2) + pow(delta_d.at(2), 2);
            double dist = sqrt(dist_squa);
            if (dist_squa == 0.0)
                // join bodies?
                exit(EXIT_FAILURE);

            double force_total = grav_const * (this->mass * other.mass) / dist_squa;
            double force_x = force_total * (delta_d.at(0)/dist);
            double force_y = force_total * (delta_d.at(1)/dist);
            double force_z = force_total * (delta_d.at(2)/dist);

            acc.at(0) += force_x/mass;
            acc.at(1) += force_y/mass;
            acc.at(2) += force_z/mass;

            //cout << name << " position: " <<pos.at(0) << ", " << pos.at(1) << "\tacceleration: " << acc.at(0) << ", " << acc.at(1) << endl;
            return delta_d;
        }

        void posUpdate(double timestep)
        {

            // Add position to history
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

        void addToCan(Mat canvas,double maxDist, double maxMass, double maxRad, bool drawpast=true)
        {
            const float scaling = 0.5;
            // Convert the space coordinates to coordinates on the canvas
            Point centre = Point((int)(min(canHeight, canWidth) * scaling * (pos.at(0)/maxDist) + 0.5 * canWidth),
                                 (int)(min(canHeight, canWidth) * scaling * (pos.at(1)/maxDist) + 0.5 * canHeight));

            // Get the radius of the circle to display, sized relative to scale
            int cirRad = (int)(0.5 + (rad/maxRad * 50)*(start_maxDist/maxDist));

            circle(canvas, centre, cirRad, this->colour, FILLED, LINE_8);

            if (!drawpast)
                return;

            // get bodies past coordinates, convert them to canvas coordinates with the current scaling
            vector<Point> pasPoints;
            for (vector<vector<double>>::iterator it = past.begin(); it != past.end(); ++it)
            {
                pasPoints.push_back(Point((int)(min(canHeight, canWidth) * scaling * (it->at(0)/maxDist) + 0.5 * canWidth),
                                          (int)(min(canHeight, canWidth) * scaling * (it->at(1)/maxDist) + 0.5 * canHeight)));
            }
            pasPoints.push_back(Point((int)(min(canHeight, canWidth) * scaling * (pos.at(0)/maxDist) + 0.5 * canWidth),
                                 (int)(min(canHeight, canWidth) * scaling * (pos.at(1)/maxDist) + 0.5 * canHeight)));
            
            polylines(canvas, pasPoints, false, colour*0.4, 1, LINE_8);
        }
};


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
    vector<Body> bodies;
    Body Earth = Body("Eath", 5.972e24, 6371, vector<double> {0, 0, 0}, vector<double> {0, 0, 0});
    Body MoonOne = Body("Moon1", 7.348e22, 1737, vector<double> {0, -200e6, 0}, vector<double>{-150.0, 0, 0});
    Body MoonTwo = Body("Moon2", 7.348e22, 1737, vector<double> {0, 200e6, 0}, vector<double>{150.0, 0, 0});
    Body MoonThree = Body("Moon3", 7.348e22, 1737, vector<double> {200e6, 0, 0}, vector<double>{0.0, -150, 0});
    Body MoonFour = Body("Moon4", 7.348e22, 1737, vector<double> {-200e6, 0, 0}, vector<double>{0.0, 150, 0});
    // bodies.push_back(Earth);
    bodies.push_back(MoonOne);
    bodies.push_back(MoonTwo);
    bodies.push_back(MoonThree);
    bodies.push_back(MoonFour);

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
    for (vector<Body>::iterator it = bodies.begin(); it != bodies.end(); ++it)
    {
        if (it->mass > maxMass)
            start_maxMass = maxMass = it->mass;

        if (it->rad > maxRad)
            start_maxRad = maxRad = it->rad;

        it->addToCan(canvas, maxDist, maxMass, maxRad);
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

        double timestep = 3600;
        for (vector<Body>::iterator lowIt = bodies.begin(); lowIt != bodies.end(); ++lowIt)
        {
            for (vector<Body>::iterator upIt = bodies.begin(); upIt != bodies.end(); ++upIt)
            {
                if (lowIt->name == upIt->name)
                    continue;

                // TODO: use lambda?
                vector<double> currDist = lowIt->accAround(*upIt);
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

        for (vector<Body>::iterator it = bodies.begin(); it != bodies.end(); ++it)
        {
            it->posUpdate(timestep);
            it->addToCan(canvas, maxDist, maxMass, maxRad);
            it->printParam();
        }
        imshow(canName, canvas);
        //waitKey(50);
        waitKey(WAITTIME);
    }

    destroyAllWindows();
    return 0;
}
