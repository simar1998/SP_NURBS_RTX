//
// Created by simar on 6/26/2023.
//

#include "CurveFitting.h"
#include "../Eigen/Core"
#include "../Eigen/Dense"
#include <iostream>

//FIXME make this work
//Implements curve fitting algo for fitting a curve using least squares method to fit a NURBS curve to points in a given param
CurveFitting::curv CurveFitting::fitCurve(std::vector<Vec3> &points) {

    std::cout<<"Num points to be fitted points: "<<std::to_string(points.size())<<std::endl;
    // First and second point in list can be defined as the inital and end control point //UNSIRE
    curv approxCurv;
    int degree = 3; //To be changed based on number of control points and knot vectors
    std::vector<GlmVec> controlPoints;//Initial control points

    //Append glm points from the BVH vec points
    controlPoints.reserve(points.size());
    for (int i = 0; i < points.size(); ++i) {
        controlPoints.push_back(GlmVec(points[i].values[0],points[i].values[1],points[i].values[2]));
    }
    approxCurv.control_points.push_back(controlPoints.front());
    approxCurv.control_points.push_back(controlPoints.back());

    //Resize number of knots available
    approxCurv.knots.resize(points.size() + degree + 1);
    for (int i = 0; i < approxCurv.knots.size(); i++) {
        approxCurv.knots[i] = static_cast<float>(i) / (approxCurv.knots.size() - 1); // uniform knot vector
    }

    //linear system to solve for the control point shifting to ensure curve normality with task
    Eigen::MatrixXf A(points.size(), controlPoints.size()); // Assume Eigen library is used
    Eigen::VectorXf b(points.size());
    for (int i = 0; i < points.size(); ++i) {
        for (int j = 0; j < controlPoints.size(); ++j) {
            float basis = computeBasisFunction(degree, approxCurv.knots, j, points[i].values[0]);
            A(i, j) = basis;
        }

        // Fill b with the values you want the curve to approximate
        b(i) = points[i].values[1]; // Assuming you want to approximate the y-values of the points
    }

    Eigen::VectorXf x = A.fullPivHouseholderQr().solve(b);

    for (int i = 0; i < controlPoints.size(); ++i) {
        // Update the control points with the solution x
        controlPoints[i][1] = x(i);
    }

    approxCurv.control_points = controlPoints;
    //Return approximated curve
    return approxCurv;
}

//Computing bassis function for Current spot on the NURBS curve
float CurveFitting::computeBasisFunction(int degree, const std::vector<float> &knotVector, int knotIndex, float parameter) {
    if (degree == 0) {
        if (parameter >= knotVector[knotIndex] && parameter < knotVector[knotIndex + 1]) {
            return 1.0f;
        } else {
            return 0.0f;
        }
    } else {
        float leftNumerator = parameter - knotVector[knotIndex];
        float leftDenominator = knotVector[knotIndex + degree] - knotVector[knotIndex];
        float leftTerm = (leftNumerator / leftDenominator) * computeBasisFunction(degree - 1, knotVector, knotIndex, parameter);

        float rightNumerator = knotVector[knotIndex + degree + 1] - parameter;
        float rightDenominator = knotVector[knotIndex + degree + 1] - knotVector[knotIndex + 1];
        float rightTerm = (rightNumerator / rightDenominator) * computeBasisFunction(degree - 1, knotVector, knotIndex + 1, parameter);

        return leftTerm + rightTerm;
    }
}
