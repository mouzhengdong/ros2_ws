#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <random>

using namespace Eigen;
using namespace std;

typedef vector<Vector3d> PointCloud;


// 生成示例点云：在原始点云基础上应用一个已知变换，作为目标点云
void generatePointCloud(PointCloud& src, PointCloud& dst, Matrix3d& R_gt, Vector3d& t_gt) {
    std::default_random_engine gen;
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    for (int i = 0; i < 100; ++i) {
        Vector3d pt(dist(gen), dist(gen), dist(gen));
        src.push_back(pt);
        dst.push_back(R_gt * pt + t_gt);
    }
}

Vector3d computeCentroid(const PointCloud& cloud) {
    Vector3d centroid = Vector3d::Zero();
    for (const auto& pt : cloud) {
        centroid += pt;
    }
    return centroid / cloud.size();
}

void icpSVD(const PointCloud& src, const PointCloud& dst, Matrix3d& R_est, Vector3d& t_est) {
    // 计算源点云和目标点云的质心
    Vector3d centroid_src = computeCentroid(src);
    Vector3d centroid_dst = computeCentroid(dst);

    // 中心化点云
    PointCloud src_centered, dst_centered;
    for (const auto& pt : src) {
        src_centered.push_back(pt - centroid_src);
    }
    for (const auto& pt : dst) {
        dst_centered.push_back(pt - centroid_dst);
    }

    // 计算协方差矩阵
    Matrix3d H = Matrix3d::Zero();
    for (size_t i = 0; i < src_centered.size(); ++i) {
        H += src_centered[i] * dst_centered[i].transpose();
    }

    // SVD分解
    JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

    // 计算旋转矩阵
    R_est = V * U.transpose();

    // 确保旋转矩阵的行列式为正
    if (R_est.determinant() < 0) {
        R_est.col(2) *= -1;
    }

    // 平移向量
    t_est = centroid_dst - R_est * centroid_src;
}


int main() {
    PointCloud src, dst;
    Matrix3d R_gt;
    Vector3d t_gt(0.5, -0.3, 0.8);

    // 绕 z 轴逆时针旋转 45° 的旋转矩阵
    R_gt = AngleAxisd(M_PI / 4, Vector3d(0, 0, 1)).toRotationMatrix();

    generatePointCloud(src, dst, R_gt, t_gt);

    Matrix3d R_est;
    Vector3d t_est;
    icpSVD(src, dst, R_est, t_est);

    cout << "\n[Ground Truth R]:\n" << R_gt << endl;
    cout << "\n[Estimated R]:\n" << R_est << endl;
    cout << "\n[Ground Truth t]: " << t_gt.transpose() << endl;
    cout << "\n[Estimated t]: " << t_est.transpose() << endl;

    return 0;
}




