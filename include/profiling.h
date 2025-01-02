#ifndef _PROFILINGH_
#define _PROFILINGH_
#include <cmath>
#include <functional>
#include <vector>

struct Point {
    double x;
    double y;
};

struct Pose {
    double x;
    double y;
    double heading;
};

struct UltraPose {
    double x;
    double y;
    double heading;
    double curvature;
};

struct Line {
    double slope;
    double yIntercept;
};

struct MPPoint {
    double x;
    double y;
    double heading;
    double linVel;
    double angVel;
    double t;
};

struct Zone {
    double startT; // start value of t
    double endT; // end value of t
    Line zoneLine; // line that represents the zone on the graph
};

struct HexicPolyData {
    double P6;
    double P5;
    double P4;
    double P3;
    double P2;
    double P1;
    double P0;
};

struct CubicPolyData {
    double a;
    double b;
    double c;
    double d;
};

struct QuadraticPolyData {
    double a;
    double b;
    double c;
};

enum Direction {
    LEFT = -1,
    STRAIGHT = 0,
    RIGHT = 1,
};

class CubicHermiteSpline {
    public:
        CubicHermiteSpline(Point startPos, Point startV, Point endPos, Point endV);
        Point findPoint(double t);
        Pose findPose(double t, double step);
        std::vector<UltraPose> entirePath(double step);
        void findFunction(void);
        void findDerivative(void);
        void findSecondDerivative(void);
        double calculateCurvature(double t);

        std::vector<CubicPolyData> functions;
        std::vector<QuadraticPolyData> derivative;
        std::vector<HexicPolyData> secondDerivative;


    private:
        double evaluateP0(double t);
        double evaluateV0(double t);
        double evaluateP1(double t);
        double evaluateV1(double t);

        Point startPos;
        Point startV;
        Point endPos;
        Point endV;
};

class MotionProfile {
    public:
        MotionProfile(std::vector<UltraPose> pointList, double maxSpeed);
        MotionProfile(std::vector<UltraPose> pointList, std::vector<std::vector<Point>> zonePoints, double maxSpeed);
        void generateVelocities(void);
        std::vector<MPPoint> profile;
        double maxSpeed;
        MPPoint findNearestPoint(double givenT);
        Direction findCurveDirectionOfPoint(MPPoint currentPoint);
        void constructWithCustomZones(std::vector<std::vector<Point>> zoneLinePoints);

    private:
        std::vector<UltraPose> pointList;
        std::vector<Zone> zones;


};

class VelocityController {
    public:
        double linVel;
        double angVel;
        std::vector<double> FINDME;
        VelocityController(double wheelDiameter, double distBetweenWheels, double gearRatio, double maxRPM);
        void startQueuedProfile(bool RAMSETE);
        void endProfile(void);
        void queueProfile(MotionProfile* profile);
        void addAction(std::function<void(void)> action, double time);


    private:
        std::vector<double> calculateOutputOfSides(double linearVelocityMPS, double angularVelocityRADPS, Direction direction);
        double calculateSingleDegree(double wheelDiameter);
        void followProfile(MotionProfile* profile, bool RAMSETE);
        
        double wheelDiameter;
        double distBetweenWheels;
        double timeToRun;
        double gearRatio;
        double maxRPM;
        MotionProfile* queuedProfile;
        std::vector<double> actionTs;
        std::vector<std::function<void(void)>> actions;
        // pros::Task* controlLoop_task_ptr = NULL;
};



#endif