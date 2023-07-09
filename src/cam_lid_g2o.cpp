#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <string>

#include <sophus/se3.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "common.h"
#include "result_verify.h"

using namespace std;

typedef pcl::PointXYZRGB PointType;
Eigen::Matrix3d inner;
string lidar_path, photo_path, intrinsic_path, extrinsic_path;
int error_threshold;
vector<float> init;

void getParameters();


// 顶点，模板参数：优化变量维度和数据类型
class Vertex : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置 override保留字表示当前函数重写了基类的虚函数
    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    // 更新
    virtual void oplusImpl(const double *update) override {
        Eigen::Matrix<double, 6, 1> updateVector;
        updateVector << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(updateVector) * _estimate;
    }

    // 存盘和读盘：留空
	virtual bool read(istream &in) {}
	virtual bool write(ostream &out) const {}
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class Edge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, Vertex> {
private:
    Eigen::Vector3d _p3d; // point 空间坐标点
    Eigen::Matrix3d _k; // 内参矩阵
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // EIGEN 内存对齐宏命令

    Edge(const Eigen::Vector3d &p3d, const Eigen::Matrix3d &k) : _p3d(p3d), _k(k)  {}

    // 计算误差
    virtual void computeError() override {
        // static_cast 将_vertices[0]的类型强制转换为const Vertex*
        const Vertex *v = static_cast<const Vertex*>(_vertices[0]); 
        const Sophus::SE3d T = v->estimate(); // 取出顶点优化变量

        Eigen::Vector3d tp3d = T * _p3d; // 旋转后的空间坐标
        // 映射到像素平面，前两维是像素坐标，/ tp3d[2]为归一化像素坐标。
        // 这里用到了k的运算，所以k要定义为Eigen的类型
        tp3d = _k * tp3d / tp3d[2]; 
        Eigen::Vector2d tp2d = tp3d.head<2>();

        _error = _measurement - tp2d; // 计算误差 _measurement为观测值
    }

    // 计算雅可比矩阵（可选）没有此函数时默认为数值推导
    // linearizeOplus函数是边（误差）对于各个顶点（优化变量）的偏导数（雅可比矩阵）的解析求导函数，
    // 若此函数不被定义，则采用G2O内置的数值求导，但是数值求导很慢，不如解析求导迅速，效果不佳，
    // 优点是不需要计算雅可比矩阵，比较方便
    virtual void linearizeOplus() override {
        const Vertex *v = static_cast<const Vertex *>(_vertices[0]);
        const Sophus::SE3d T = v->estimate(); // 取出上次估计的T
        Eigen::Vector3d tp3d = T * _p3d;

        double invZ = 1 / tp3d[2], invZ2 = invZ * invZ;
        double fx = _k(0, 0);
        double fy = _k(1, 1);
        double cx = _k(0, 2);
        double cy = _k(1, 2);

        _jacobianOplusXi << -fx * invZ,
                            0,
                            fx * tp3d[0] * invZ2,
                            fx * tp3d[0] * tp3d[1] * invZ2,
                            -fx - -fx * tp3d[0] * tp3d[0] * invZ2,
                            fx * tp3d[1] * invZ,
                            0,
                            -fy * invZ,
                            fy * tp3d[1] * invZ2,
                            fy + fy * tp3d[1] * tp3d[1] * invZ2,
                            -fy * tp3d[0] * tp3d[1] * invZ2,
                            -fy * tp3d[0] * invZ;

    }
    virtual bool read(istream &in) {}
	virtual bool write(ostream &out) const {}
};

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_lidar_path", lidar_path)) {
        cout << "Can not get the value of input_lidar_path" << endl;
        exit(1);
    }
    if (!ros::param::get("input_photo_path", photo_path)) {
        cout << "Can not get the value of input_photo_path" << endl;
        exit(1);
    }
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("extrinsic_path", extrinsic_path)) {
        cout << "Can not get the value of extrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("error_threshold", error_threshold)) {
        cout << "Can not get the value of error_threshold" << endl;
        exit(1);
    }
    init.resize(12);
    if (!ros::param::get("init_value", init)) {
        cout << "Can not get the value of init_value" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "getExt1");
    getParameters();

    vector<PnPData> pData;
    getData(lidar_path, photo_path, pData);

    Eigen::Matrix4d extrin;
    // set the intrinsic parameters
    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    inner << intrinsic[0], intrinsic[1], intrinsic[2],
    intrinsic[3], intrinsic[4], intrinsic[5],
    intrinsic[6], intrinsic[7], intrinsic[8];

    // init the matrix of extrinsic, matrix of rotation and translation
    // keep this init value, or it is possible to get a local optimal result
    Eigen::Matrix3d R;  
    R << init[0], init[1], init[2],
         init[4], init[5], init[6],
         init[8], init[9], init[10];
    Eigen::Quaterniond q(R); 
    double ext[7];
    ext[0] = q.x();
    ext[1] = q.y();
    ext[2] = q.z();
    ext[3] = q.w();
    ext[4] = init[3];  
    ext[5] = init[7];  
    ext[6] = init[11];  

    // Eigen::Map: 一个模板函数，用于将一串数据映射到一个矩阵或者向量中。
    Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
    Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);


    //g2o
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> BlockSolverType;  //优化变量维度为6，误差维度为2
	typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

    // 梯度下降方法
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true); // 打开调试输出

    // 构造顶点
    Vertex *v = new Vertex();
    v->setEstimate(Sophus::SE3d(m_q, m_t)); // 使用ceres优化值初始化
    v->setId(0);
    optimizer.addVertex(v);

    vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> ps2d;
	vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ps3d;
    for(auto val : pData) {
        ps2d.push_back(Eigen::Vector2d(val.u, val.v));
        ps3d.push_back(Eigen::Vector3d(val.x, val.y, val.z));
    }

    // 构造边
    for (int i = 0; i < (int)ps3d.size(); i++) {
        Edge *edge = new Edge(ps3d[i], inner);
        edge->setId(i);
        edge->setVertex(0, v); // 设置连接的顶点
        edge->setMeasurement(ps2d[i]); // 观测值
        // 信息矩阵：协方差矩阵之逆，这里没有，填单位矩阵，维度与误差变量一致
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    // 执行优化
    optimizer.initializeOptimization();
    optimizer.optimize(10); // 迭代次数

    // 输出优化值
    Sophus::SE3d T = v->estimate();
    cout << "estimated model: " << endl << T.matrix() << endl;



    

    // Eigen::Matrix3d rot = m_q.toRotationMatrix();
    // writeExt(extrinsic_path, rot, m_t);
    // cout << rot << endl;
    // cout << m_t << endl;

    // cout << "Use the extrinsic result to reproject the data" << endl;
    // float error[2] = {0, 0};
    // getUVError(intrinsic_path, extrinsic_path, lidar_path, photo_path, error, error_threshold);    
    
    // cout << "u average error is: " << error[0] << endl;
    // cout << "v average error is: " << error[1] << endl;
    // if (error[0] + error[1] < error_threshold) {
    //     cout << endl << "The reprojection error smaller than the threshold, extrinsic result seems ok" << endl;
    // }
    // else {
    //     cout << endl << "The reprojection error bigger than the threshold, extrinsic result seems not ok" << endl;
    // }

    return 0;  
}


