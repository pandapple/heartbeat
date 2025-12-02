#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include <thread>
#include <chrono>

using namespace std;
using namespace pcl;

// 定义点类型
typedef PointXYZRGB PointT;
typedef PointCloud<PointT> PointCloudT;

// ================================
// 1. 定义隐式 3D 爱心方程（与Python文件一致）
// ================================
double heart_func(double x, double y, double z) {
    double term1 = x*x + (9.0/4.0)*y*y + z*z - 1.0;
    double term2 = pow(term1, 3);
    double term3 = x*x * pow(z, 3);
    double term4 = (9.0/80.0) * y*y * pow(z, 3);
    return term2 - term3 - term4;
}

// 三角形结构
struct Triangle {
    double v0[3], v1[3], v2[3];
    
    Triangle(double x0, double y0, double z0, 
             double x1, double y1, double z1,
             double x2, double y2, double z2) {
        v0[0] = x0; v0[1] = y0; v0[2] = z0;
        v1[0] = x1; v1[1] = y1; v1[2] = z1;
        v2[0] = x2; v2[1] = y2; v2[2] = z2;
    }
    
    // 计算三角形面积
    double area() const {
        double e1[3] = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
        double e2[3] = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
        // 叉积
        double cross[3] = {
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0]
        };
        double norm = sqrt(cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2]);
        return 0.5 * norm;
    }
};

// 使用线性插值在边上找到曲面交点
bool interpolateEdge(double x1, double y1, double z1, double f1,
                     double x2, double y2, double z2, double f2,
                     double& x, double& y, double& z) {
    if (f1 * f2 > 0) return false;  // 同号，没有交点
    if (abs(f1 - f2) < 1e-10) return false;
    
    double t = -f1 / (f2 - f1);
    x = x1 + t * (x2 - x1);
    y = y1 + t * (y2 - y1);
    z = z1 + t * (z2 - z1);
    return true;
}

// ================================
// 2. Marching Cubes 网格提取（简化版本）
// ================================
void extract_mesh(int resolution, vector<Triangle>& triangles) {
    // 空间范围
    double scale = 1.5;
    double x_min = -scale, x_max = scale;
    double y_min = -scale, y_max = scale;
    double z_min = -scale, z_max = scale;
    
    double dx = (x_max - x_min) / resolution;
    double dy = (y_max - y_min) / resolution;
    double dz = (z_max - z_min) / resolution;
    
    cout << "步骤1: 提取隐式曲面网格（Marching Cubes方法）..." << endl;
    
    // 遍历所有网格单元
    for (int i = 0; i < resolution; i++) {
        double x = x_min + i * dx;
        for (int j = 0; j < resolution; j++) {
            double y = y_min + j * dy;
            for (int k = 0; k < resolution; k++) {
                double z = z_min + k * dz;
                
                // 计算单元8个顶点的函数值
                double f000 = heart_func(x, y, z);
                double f001 = heart_func(x, y, z + dz);
                double f010 = heart_func(x, y + dy, z);
                double f011 = heart_func(x, y + dy, z + dz);
                double f100 = heart_func(x + dx, y, z);
                double f101 = heart_func(x + dx, y, z + dz);
                double f110 = heart_func(x + dx, y + dy, z);
                double f111 = heart_func(x + dx, y + dy, z + dz);
                
                // 检查是否有符号变化
                double f_min = min({f000, f001, f010, f011, f100, f101, f110, f111});
                double f_max = max({f000, f001, f010, f011, f100, f101, f110, f111});
                
                if (f_min * f_max >= 0) continue;  // 没有符号变化，曲面不穿过这个单元
                
                // 收集边上的交点（检查12条边）
                vector<vector<double>> edge_points;
                double px, py, pz;
                
                // 边1-4: z=z0 平面的4条边
                if (interpolateEdge(x, y, z, f000, x+dx, y, z, f100, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x+dx, y, z, f100, x+dx, y+dy, z, f110, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x+dx, y+dy, z, f110, x, y+dy, z, f010, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x, y+dy, z, f010, x, y, z, f000, px, py, pz))
                    edge_points.push_back({px, py, pz});
                
                // 边5-8: z=z0+dz 平面的4条边
                if (interpolateEdge(x, y, z+dz, f001, x+dx, y, z+dz, f101, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x+dx, y, z+dz, f101, x+dx, y+dy, z+dz, f111, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x+dx, y+dy, z+dz, f111, x, y+dy, z+dz, f011, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x, y+dy, z+dz, f011, x, y, z+dz, f001, px, py, pz))
                    edge_points.push_back({px, py, pz});
                
                // 边9-12: 连接上下平面的4条垂直边
                if (interpolateEdge(x, y, z, f000, x, y, z+dz, f001, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x+dx, y, z, f100, x+dx, y, z+dz, f101, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x+dx, y+dy, z, f110, x+dx, y+dy, z+dz, f111, px, py, pz))
                    edge_points.push_back({px, py, pz});
                if (interpolateEdge(x, y+dy, z, f010, x, y+dy, z+dz, f011, px, py, pz))
                    edge_points.push_back({px, py, pz});
                
                // 如果有3个或更多交点，形成三角形（简化：使用前3个点）
                if (edge_points.size() >= 3) {
                    // 使用前3个点形成三角形（实际marching cubes会更复杂，这里简化处理）
                    for (size_t idx = 0; idx < edge_points.size() - 2; idx++) {
                        triangles.push_back(Triangle(
                            edge_points[0][0], edge_points[0][1], edge_points[0][2],
                            edge_points[idx+1][0], edge_points[idx+1][1], edge_points[idx+1][2],
                            edge_points[idx+2][0], edge_points[idx+2][1], edge_points[idx+2][2]
                        ));
                    }
                }
            }
        }
        if ((i + 1) % 20 == 0) {
            cout << "进度: " << (i + 1) * 100 / resolution << "%" 
                 << " (已提取 " << triangles.size() << " 个三角形)" << endl;
        }
    }
}

// ================================
// 3. 按三角形面积均匀采样点（与Python文件一致）
// ================================
void sample_points_from_mesh(const vector<Triangle>& triangles, int n_points, PointCloudT::Ptr cloud) {
    if (triangles.empty()) return;
    
    cout << "步骤2: 从三角形网格上均匀采样点..." << endl;
    cout << "提取了 " << triangles.size() << " 个三角形，将采样 " << n_points << " 个点" << endl;
    
    // 计算每个三角形的面积
    vector<double> areas;
    double total_area = 0.0;
    for (const auto& tri : triangles) {
        double a = tri.area();
        areas.push_back(a);
        total_area += a;
    }
    
    if (total_area < 1e-10) return;
    
    // 构建累积分布函数（CDF）
    vector<double> cdf;
    double cumsum = 0.0;
    for (double a : areas) {
        cumsum += a / total_area;
        cdf.push_back(cumsum);
    }
    
    // 随机数生成器
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);
    
    // 采样点
    for (int i = 0; i < n_points; i++) {
        // 根据面积分布选择三角形
        double r = dis(gen);
        int tri_idx = 0;
        for (size_t j = 0; j < cdf.size(); j++) {
            if (r <= cdf[j]) {
                tri_idx = j;
                break;
            }
        }
        
        const Triangle& tri = triangles[tri_idx];
        
        // 使用重心坐标在三角形内采样（与Python文件一致）
        double u = dis(gen);
        double v = dis(gen);
        if (u + v > 1.0) {
            u = 1.0 - u;
            v = 1.0 - v;
        }
        double w = 1.0 - u - v;
        
        // 计算采样点坐标
        PointT point;
        point.x = w * tri.v0[0] + u * tri.v1[0] + v * tri.v2[0];
        point.y = w * tri.v0[1] + u * tri.v1[1] + v * tri.v2[1];
        point.z = w * tri.v0[2] + u * tri.v1[2] + v * tri.v2[2];
        
        // 设置颜色为粉色
        point.r = 255;
        point.g = 192;
        point.b = 203;
        
        cloud->points.push_back(point);
    }
}

// 生成3D爱心点云（使用Marching Cubes + 均匀采样，与Python文件策略一致）
PointCloudT::Ptr generateHeartCloud(int resolution = 160, int n_points = 30000) {
    PointCloudT::Ptr cloud(new PointCloudT);
    
    // 步骤1: 提取网格
    vector<Triangle> triangles;
    extract_mesh(resolution, triangles);
    
    // 步骤2: 从网格上均匀采样点
    sample_points_from_mesh(triangles, n_points, cloud);
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    
    return cloud;
}

// 计算点到原点的距离，用于扩展方向
double distanceFromOrigin(const PointT& point) {
    return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

// 获取从原点到点的单位方向向量
void getDirectionVector(const PointT& point, double& dx, double& dy, double& dz) {
    double dist = distanceFromOrigin(point);
    if (dist > 0.001) {
        dx = point.x / dist;
        dy = point.y / dist;
        dz = point.z / dist;
    } else {
        dx = 1.0;
        dy = 0.0;
        dz = 0.0;
    }
}

int main(int argc, char** argv) {
    // 生成爱心点云（所有采样点都会参与心跳动画）
    cout << "生成3D爱心点云（使用Marching Cubes + 均匀采样）..." << endl;
    PointCloudT::Ptr heart_cloud = generateHeartCloud(160, 30000);
    cout << "生成了 " << heart_cloud->points.size() << " 个采样点" << endl;
    cout << "所有采样点将参与心跳动画" << endl;
    
    // 保存原始点位置（用于心跳动画）
    PointCloudT::Ptr original_cloud(new PointCloudT);
    *original_cloud = *heart_cloud;  // 复制原始位置
    
    // 创建可视化器
    visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Heart Beat"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    viewer->addCoordinateSystem(2.0);  // 坐标系统大小
    viewer->initCameraParameters();
    
    // 添加爱心点云（所有采样点，粉色，较大）
    visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(heart_cloud);
    viewer->addPointCloud<PointT>(heart_cloud, rgb_handler, "heart_cloud");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "heart_cloud");  // 增大点大小以便观察
    
    // 设置相机位置 - 调整到更合适的位置以观察完整的爱心
    viewer->setCameraPosition(3, 3, 3, 0, 0, 0, 0, 0, 1);
    
    cout << "开始心跳动画..." << endl;
    cout << "按 'q' 键退出" << endl;
    
    // 心跳动画参数
    double expansion_factor = 0.3;  // 扩展距离
    double animation_speed = 0.05;  // 动画速度
    double time = 0.0;
    
    // 主循环
    while (!viewer->wasStopped()) {
        // 更新所有采样点的位置（心跳动画）
        for (size_t i = 0; i < heart_cloud->points.size(); i++) {
            const PointT& original = original_cloud->points[i];
            PointT& current = heart_cloud->points[i];
            
            // 计算扩展方向（从原点到点的方向）
            double dx, dy, dz;
            getDirectionVector(original, dx, dy, dz);
            
            // 使用正弦波实现周期性扩展和收缩
            double phase = sin(time * 2.0 * M_PI);  // 周期为1秒
            double offset = expansion_factor * phase;
            
            // 更新位置
            current.x = original.x + dx * offset;
            current.y = original.y + dy * offset;
            current.z = original.z + dz * offset;
        }
        
        // 更新时间
        time += animation_speed;
        
        // 更新点云
        viewer->updatePointCloud<PointT>(heart_cloud, rgb_handler, "heart_cloud");
        
        // 刷新视图
        viewer->spinOnce(50);  // 50ms延迟，约20fps
        
        // 短暂休眠以控制动画速度
        this_thread::sleep_for(chrono::milliseconds(50));
    }
    
    return 0;
}
