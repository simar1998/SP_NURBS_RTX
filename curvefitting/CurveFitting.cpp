//
// Created by simar on 6/26/2023.
//

#include "CurveFitting.h"
#include "../Eigen/Core"
#include <iostream>

//FIXME make this shit work fam
//Implements curve fitting algo for fitting a curve using linear regression to fit a NURBS curve to points in a given param
CurveFitting::curv CurveFitting::fitCurve(std::vector<Vec3> &points) {

    std::cout<<"Num points to be fitted points: "<<std::to_string(points.size())<<std::endl;
    // First and second point in list can be defined as the inital and end control point //UNSIRE
    curv approxCurv;
    int degree = 3; //To be changed based on number of control points and knot vectors
    std::vector<GlmVec> controlPoints;//Intial control points

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
        approxCurv.knots[i] = (float) i / (approxCurv.knots.size() - 1); // uniform knot vector
    }

    //Work on linear system to solve for the control point shifting to ensure curve normality with task
    Eigen::MatrixXf A(points.size(), controlPoints.size()); // Assume Eigen library is used
    Eigen::VectorXf b(points.size());
    // fill A and b based on your problem specifics

    //FIXME
    // Solve for the control points and stop being a bitch
    Eigen::VectorXf x = A.fullPivHouseholderQr().solve(b);



    //Return approximated curve
    return approxCurv;
}
