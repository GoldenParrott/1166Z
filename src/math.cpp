#include "init.h"

// distance formula function
double calculateDistance(Coordinate point1, Coordinate point2) {
    return std::sqrt(std::pow((point2.x - point1.x), 2) + std::pow((point2.y - point1.y), 2));
}

// calculates the standard deviation of a set of values, used for the filter (private)
double calculateStandardDeviation(
    std::deque<double> listOfValues // list of values to use
)
{
    // calculates the mean of the values
    double meanOfValues = 0;
    for (int i = 0; i < listOfValues.size(); i++) {
        meanOfValues += listOfValues[i]; // adds each value together to calculate their mean
    }
    meanOfValues = meanOfValues / listOfValues.size();

    // calculates the variance of the values
    std::vector<double> listOfDifferences;
    double variance = 0;
    for (int i = 0; i < listOfValues.size(); i++) {
        variance += std::pow((listOfValues[i] - meanOfValues), 2); // variance is calculated by adding the squares of each value's difference from the mean together
    }
    variance = variance / listOfValues.size();

    double standardDeviation = std::sqrt(variance); // standard deviation = square root of variance

    return standardDeviation;
}

Line calculatePerpendicular(
    Coordinate point1, // has a line between it and point2 that will have a line perpendicular to it
    Coordinate point2 // the point that the perpendicular line will cross
)
{
    Line Line1;
    Line LinePerp;

    if (point2.y - point1.y == 0) { // handles the case if y does not change (line takes the form of x = k)
        LinePerp.slope = NAN;
        LinePerp.yIntercept = point2.x; // yIntercept serves as the value of k (in x = k) for this case
        LinePerp.equality = point1.x > point2.x // a copy of findEquality that simply compares the x-values of both points to determine the equality
                                ? 2 // if the value of the first point is greater than the value of the second point, 
                                     // then the equality is on the opposite side (negative)
                                : -2;
        return LinePerp;
    }
    else if (point2.x - point1.x == 0) { // handles the case if x does not change (line takes the form of y = k)
        LinePerp.slope = 0;
        LinePerp.yIntercept = point2.y; // yIntercept serves as the value of k (in y = k) for this case
        LinePerp.equality = point1.y > point2.y // a copy of findEquality that simply compares the x-values of both points to determine the equality
                                ? 2 // if the value of the first point is greater than the value of the second point, 
                                     // then the equality is on the opposite side (negative)
                                : -2;
        return LinePerp;
    }
    
    // handles all y = mx + b cases
    Line1.slope = (point2.y - point1.y) / (point2.x - point1.x); // calculates the slope of the first line
    Line1.yIntercept = (-1 * (Line1.slope * point1.x)) + point1.y; // formula for y-intercept based on point-slope form
    Line1.equality = 0; // this line is just treated as an equation
    

    LinePerp.slope = -1 * (1 / Line1.slope); // slope of a perpendicular line is the negative reciprocal of the slope of the original
    LinePerp.yIntercept = (-1 * (LinePerp.slope * point2.x)) + point2.y;
    LinePerp.equality = findEquality(LinePerp, point1); // sets the inequality to include the side with point 1 on it

    return LinePerp;
}

int findEquality(
    Line line, // the line to find the equality of
    Coordinate includedPoint // a point on the side of the inequality that will be part of the inequality
)
{
    double result = (line.slope * includedPoint.x) + line.yIntercept; // the result of the equation when x from the point is plugged into it

    if (includedPoint.y != result) { // if the actual y-value is not the same as the y-value that the line produces, they are not equal
        if (includedPoint.y > result) { // if the actual y-value is greater than the line's y-value, then the included part is above the line
            return 2; // 2 signifies greater than for the Line structure
        }
        else { // if the actual y-value is less than the line's y-value, then the included part is below the line
            return -2; // -2 signifies less than for the Line structure
        }
    } else { // if the point is on the line, then the line is an equation
            return 0; // 0 signifies equal to on the Line structure
    }
}