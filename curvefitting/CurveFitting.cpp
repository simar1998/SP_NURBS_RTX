#include "CurveFitting.h"
#include "../Eigen/Core"
#include "../Eigen/Dense"
#include <iostream>
CurveFitting::curv CurveFitting::fitCurve(const std::vector<Vec3>& points) {
    if (points.empty()) {
        throw std::invalid_argument("Empty vector of points");
    }
    std::cout << "Num points to be fitted points: " << points.size() << std::endl;
    curv approxCurv;
    int degree = 3;
    std::vector<GlmVec> controlPoints;
    controlPoints.reserve(points.size());
    for (const auto& point : points) {
        controlPoints.emplace_back(point.values[0], point.values[1], point.values[2]);
    }
    approxCurv.control_points.push_back(controlPoints.front());
    approxCurv.control_points.push_back(controlPoints.back());
    approxCurv.knots.resize(points.size() + degree + 1);
    for (int i = 0; i < approxCurv.knots.size(); i++) {
        approxCurv.knots[i] = static_cast<float>(i) / (approxCurv.knots.size() - 1);
    }
    Eigen::MatrixXf A(points.size(), controlPoints.size());
    Eigen::VectorXf b(points.size());
    for (int i = 0; i < points.size(); ++i) {
        for (int j = 0; j < controlPoints.size(); ++j) {
            float basis = computeBasisFunction(degree, approxCurv.knots, j, points[i].values[0]);
            A(i, j) = basis;
        }
        b(i) = points[i].values[1];
    }
    Eigen::VectorXf x;
    try {
        x = A.fullPivHouseholderQr().solve(b);
    } catch (const std::runtime_error& e) {
        throw std::runtime_error("Error solving linear system: " + std::string(e.what()));
    }
    for (int i = 0; i < controlPoints.size(); ++i) {
        controlPoints[i][1] = x(i);
    }
    approxCurv.control_points = controlPoints;
    return approxCurv;
}
float CurveFitting::computeBasisFunction(int degree, const std::vector<float>& knotVector, int knotIndex, float parameter) {
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