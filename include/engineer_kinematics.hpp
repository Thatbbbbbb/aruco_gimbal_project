#include<vector>
#include <cmath>
#include <array>
#include <limits>
#include<iostream>
#include <eigen3/Eigen/Dense>
// #include "engineer_serial.h"

namespace hitcrt{
namespace kinematics{

struct params{
    float delta = M_PI_2;//初始状态下在上平台投影点的夹角,用于使用我们定义的坐标系去计算V

    float yita1 = -M_PI_2;//初始W轴角度
    float yita2 = 30.0/180.0*M_PI; 
    float yita3 = 150.0/180.0*M_PI;

    float arfa1 = 54.0/180.0*M_PI;//连杆参量
    float arfa2 = M_PI_2;
};

class engineer_kinematics
{
private:
    /* data */
    float delta_ = M_PI_2;//初始状态下在上平台投影点的夹角

    float yita_[3];//初始状态下上平台三个投影点对应的角度
    
    float arfa1_;
    float arfa2_;

    inline bool get_3v_fromT(const Eigen::Isometry3d &T,
                        Eigen::Vector3d &vector_V1,Eigen::Vector3d &vector_V2,Eigen::Vector3d &vector_V3);
    inline bool solve_oneangle(const Eigen::Vector3d &vector_V,const size_t& index,std::array<float, 2>& angles);//从一个固定的上平台向量中反解出电机角度
    inline bool clamp_to_half_pi(float& angle);
    inline bool is_final_solution(const std::array<float, 3>& joints);

public:
    engineer_kinematics(const params& params_temp);//默认使用工程的参数
    ~engineer_kinematics() = default;

    // Eigen::Isometry3d forward_kinematics(const std::vector<float>& joints);
    bool inverse_kinematics(const Eigen::Isometry3d &T,std::vector<std::array<float, 3>>& res_all);

    // float solve_triangle(const float &a,const float &b,const float &c);//注意处理共线的情况哦

};

engineer_kinematics::engineer_kinematics(const params& params_temp){
    delta_ = params_temp.delta;

    yita_[0] = params_temp.yita1;
    yita_[1] = params_temp.yita2;
    yita_[2] = params_temp.yita3;

    arfa1_ = params_temp.arfa1;
    arfa2_ = params_temp.arfa2;
}

// Eigen::Isometry3d engineer_kinematics::forward_kinematics(const std::vector<float>& joints){

// }

bool engineer_kinematics::inverse_kinematics(const Eigen::Isometry3d &T,std::vector<std::array<float, 3>>& res_all){
    Eigen::Vector3d vector_V1;
    Eigen::Vector3d vector_V2;
    Eigen::Vector3d vector_V3;
    // std::cout<<"输入的矩阵\n"<<T.matrix()<<std::endl;

    bool flag1 = get_3v_fromT(T,vector_V1,vector_V2,vector_V3);
    // std::cout<<"算出来的三个向量\n"<<vector_V1<<"\n第二个\n"<<vector_V2<<"\n第三个\n"<<vector_V3<<std::endl;

    if(!flag1) {
        return false;
    }

    std::array<float, 2> angle1{};
    std::array<float, 2> angle2{};
    std::array<float, 2> angle3{};//电机的三个角度值,每个都有可能有两个解

    bool flag2 = solve_oneangle(vector_V1,0,angle1);

    if(!flag2){
        std::cout<<"是sita1没有解"<<std::endl;
        return false;
    } 

    flag2 = solve_oneangle(vector_V2,1,angle2);
    
    if(!flag2){
        std::cout<<"是sita2没有解"<<std::endl;
        return false;
    } 
    flag2 = solve_oneangle(vector_V3,2,angle3);
    
    if(!flag2){
        std::cout<<"是sita3没有解"<<std::endl;
        return false;
    } 
    res_all.clear();
    for(size_t i = 0; i < 2; ++i){
        for(size_t j = 0; j < 2; ++j){
            for(size_t k = 0; k < 2; ++k){
                if(!is_final_solution({angle1[i], angle2[j], angle3[k]})){
                    continue;
                }
                
                float joint_1 = angle1[i];
                clamp_to_half_pi(joint_1);
                float joint_2 = angle2[j] - angle1[i];
                clamp_to_half_pi(joint_2);
                float joint_3 = angle3[k] - angle2[j];
                clamp_to_half_pi(joint_3);


                res_all.push_back({joint_1, joint_2, joint_3});
            }
        }
    }

    return !res_all.empty();
}



inline bool engineer_kinematics::get_3v_fromT(const Eigen::Isometry3d &T,
                    Eigen::Vector3d &vector_V1,Eigen::Vector3d &vector_V2,Eigen::Vector3d &vector_V3){
    float angle_V1 = yita_[0] + delta_;
    float angle_V2 = yita_[1] + delta_;
    float angle_V3 = yita_[2] + delta_;
    // std::cout<<"三个角度的值1:"<<angle_V1*180*M_1_PI<<"第二个角度:"<<angle_V2*180*M_1_PI<<"第三个角度:"<<angle_V3*180*M_1_PI<<std::endl;
    auto X_axis = T.rotation().col(0);
    auto Y_axis = T.rotation().col(1);
    // auto Z_axis = T.rotation().col(2);

    vector_V1 = X_axis * std::cos(angle_V1) + Y_axis * std::sin(angle_V1);
    vector_V1.normalize();
    vector_V2 = X_axis * std::cos(angle_V2) + Y_axis * std::sin(angle_V2);
    vector_V2.normalize();
    vector_V3 = X_axis * std::cos(angle_V3) + Y_axis * std::sin(angle_V3);
    vector_V3.normalize();//减少浮点误差带来的影响

    return true;

}
//把角度映射到-M_PI_2   M_PI_2
inline bool engineer_kinematics::clamp_to_half_pi(float& angle){
    const float tol = 1e-8;
    for(int k = -2; k <= 2; ++k){
        float candidate = angle + k * 2 * M_PI;
        if(candidate >= -M_PI_2 - tol && candidate <= M_PI_2 + tol){
            angle = candidate;
            return true;
        }
    }
    return false;
}

//输出的角度都是还没有映射的,因为等一下还要处理,干脆最后干
inline bool engineer_kinematics::solve_oneangle(const Eigen::Vector3d &vector_V,const size_t& index,std::array<float, 2>& angles){
    if(index >= 3 ){
        return false;
    }
    //从一个固定的上平台向量中反解出电机角度
    std::array<float, 2> tan_angle_2 = {0.0, 0.0};
    size_t root_count = 0;
    float a = vector_V.z() * std::cos(arfa1_) + std::cos(arfa2_) + vector_V.x() * std::sin(arfa1_);
    float b = -2 * vector_V.y() * std::sin(arfa1_)   ;
    float c = vector_V.z() * std::cos(arfa1_) + std::cos(arfa2_) - vector_V.x() * std::sin(arfa1_);

    // std::endl
    // std::cout<<"检查一下sin(arfa1):"<<std::sin(arfa1_)<<" cos(arfa2_):"<<std::cos(arfa2_)<<std::endl;
    // std::cout<<"这次的向量是\n"<<vector_V<<std::endl;
    // std::cout<<"二次方程的三个系数1:"<<a<<" 2:"<<b<<" 3:"<<c<<std::endl;
    if(std::abs(a) < 1e-5){
        std::cout<<"index:"<<index<<"这个遇到sin为0的情况了"<<std::endl;

        float angle_first = M_PI;
        angles[0] = angle_first - yita_[index]; //减去W的初始角度

        float angle_second = -M_PI;
        angles[1] = angle_second - yita_[index];

        return true;
    }else {
        float delta_fun = b * b - 4 * a * c;
        if(std::abs(delta_fun) < 1e-5 ){//说明是相等的解
            tan_angle_2[0] = ( -b )/(2 * a);
            root_count = 1;
        }else if(delta_fun<0) {
            std::cout<<"这次逆运动学没有解"<<std::endl;
            return false;
        }else{
            float sqrt_delta = std::sqrt(delta_fun);
            tan_angle_2[0] = (-b - sqrt_delta)/(2 * a);
            tan_angle_2[1] = (-b + sqrt_delta)/(2 * a);
            root_count = 2;
        }
    }

    if(root_count == 0){
        return false;
    }

    for(size_t i = 0; i < root_count; ++i){
        float angle = std::atan2(tan_angle_2[i],1.0);//范围是-pi/2到pi/2
        angle = angle * 2 - yita_[index]; //减去W的初始角度

        angles[i] = angle;
    }
    if(root_count == 1){
        angles[1] = angles[0];
    }
    return true;
}

inline float normalize_angle_diff(float diff) {
    while (diff <= -M_PI - 1e-5) diff += 2.0 * M_PI;
    while (diff > M_PI + 1e-5)   diff -= 2.0 * M_PI;
    return diff;
}

//判断是不是顺时针的
inline bool engineer_kinematics::is_final_solution(const std::array<float, 3>& joints){
    // float J1 = normalize_angle_diff(joints[0]);
    // float J2 = normalize_angle_diff(joints[1]);
    // float J3 = normalize_angle_diff(joints[2]);

    // std::cout<<"输入的是1: "<<joints[0]*180*M_1_PI<<" 2:"<<joints[1]*180*M_1_PI<<" 3: "<<joints[2]*180*M_1_PI<<std::endl;
    float d12  = normalize_angle_diff(joints[1] - joints[0]);
    float d13  = normalize_angle_diff(joints[2] - joints[0]);
    float d23  = normalize_angle_diff(joints[2] - joints[1]);

    // std::cout<<"输入的是d12: "<<d12<<"d13: "<<d13<<" d23:"<<d23<<std::endl;
    // std::cout<<std::endl;


    if(d12 < -1e-5 ){
        return false;
    }

    if(d13 < -1e-5){
        return false;
    }

    if(d23 < -1e-5){
        return false;
    }
    // std::cout<<"pass"<<std::endl;
    return true;
}

};//kinematics
};//hitcrt
