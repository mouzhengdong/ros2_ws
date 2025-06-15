#include <iostream>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/manifold.h"

// 构造 y = exp(ax^2 + bx + c)
struct CostFunctor {
    CostFunctor(double x, double y) : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const abc, T* residual) const {
        residual[0] = T(y_) - ceres::exp(abc[0]*T(x_)*T(x_) + abc[1]*T(x_) + abc[2]);
        return true;
    }

private:
    const double x_, y_;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 构造问题
    ceres::Problem problem;
    double abc[3] = {1.0, 1.0, 1.0};  // 初始估计

    for (double x = 0; x < 1; x += 0.1) {
        double y = std::exp(2 * x * x + 3 * x + 1);
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CostFunctor, 1, 3>(
                new CostFunctor(x, y)),
            nullptr,
            abc);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
    std::cout << "Estimated params: " << abc[0] << ", " << abc[1] << ", " << abc[2] << "\n";
    return 0;
}
