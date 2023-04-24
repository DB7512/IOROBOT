#include "robotcontrol.h"
#include <fstream>
#include <iostream>
#include <QDebug>
using namespace std;

#define PI 3.14159265358979
RobotControl::RobotControl(QObject *parent)
    : QObject{parent}
{
    m_connectstate = false;
    m_massagestate = false;
    m_getpointstate = false;
    IdentityMatrix(m_tcpoffsetmatrix);
    IdentityMatrix(m_baseoffsetmatrix);
    SetTcpOffsetFromFile("/home/hua/IOROBOT/MassageRobot/TcpOffset.txt");
    SetBaseOffsetFromFile("/home/hua/IOROBOT/MassageRobot/BaseOffset.txt");
    for(int i = 0; i < 6; i++) {
        m_tcpoffset[i] = 0.0;
        m_baseoffset[i] = 0.0;
    }

}


RobotControl::~RobotControl()
{
    m_massagestate = false;
}

int RobotControl::CreatThread()
{
    int ret = pthread_create(&m_control, 0, RobotControl_thread, this);
    return ret;
}

void *RobotControl::RobotControl_thread(void *arg)
{
    RobotControl *robot = (RobotControl*)arg;
    vector<vector<float> > acupoint;
    vector<vector<float> > run_point;
    int acupoint_number = 0;
    int run_point_number = 0;
    vector<float> temp_point;
    vector<float> final_point;
    float point[3] = {0.0};
    float ftemp_point[6] = {0.0};
    int temp_number = 0;
    run_point.clear();
    temp_point.clear();
    qDebug("1");
    while(robot->m_massagestate) {
        qDebug("3");
        if(!robot->m_getpointstate) {
            if(robot->ReadPointFromTXT("/home/hua/IOROBOT/MassageRobot/Acupoint.txt",acupoint)) {
                acupoint_number = size(acupoint);
                for(int i = 0; i < acupoint_number*3; i++) {
                    temp_number = i/3;
                    temp_point = acupoint.at(temp_number);
                    for(int j = 0; j < 3; j++) {
                        point[j] = temp_point.at(j);
                    }
                    temp_point.clear();
                    if(i%3 == 0 || i%3 == 2) {
                        point[0] -= 100;
                        robot->ProcessPosition(point, final_point);
                        run_point.push_back(final_point);
                        final_point.clear();
                    } else {
                        robot->ProcessPosition(point, final_point);
                        run_point.push_back(final_point);
                        final_point.clear();
                    }
                }
                robot->m_getpointstate = true;
            } else {
                printf("获取穴位信息失败！\n");
            }
        } else {
            run_point_number = size(run_point);
            for(int i = 0; i < run_point_number; i++) {
                temp_point = run_point.at(i);
                temp_number = size(temp_point);
                for(int j = 0; j < temp_number; j++) {
                    ftemp_point[j] = temp_point.at(j);
                }
                robot->m_arm->set_position(ftemp_point,false);
                if(robot->m_emergencystate) {
                    robot->m_arm->reset(true);
                    printf("发生碰撞，请注意安全！\n");
                    robot->m_massagestate = false;
                    robot->m_getpointstate = false;
                    break;
                }
            }
        }
    }

//    RobotControl *robot = (RobotControl*)arg;
//    vector<vector<float> > acupoint;
//    vector<vector<float> > run_point;
//    int acupoint_number = 0;
//    int run_point_number = 0;
//    vector<float> temp_point(3);
//    float point[3] = {0.0};
//    int temp_number = 0;
//    run_point.clear();
//    temp_point.clear();
//    while(robot->m_massagestate) {
//        if(!robot->m_getpointstate) {
//            if(robot->ReadPointFromTXT("/home/hua/IOROBOT/MassageRobot/Acupoint.txt",acupoint)) {
//                acupoint_number = size(acupoint);
//                for(int i = 0; i < acupoint_number*3; i++) {
//                    temp_number = i/3;
//                    temp_point = acupoint.at(temp_number);
//                    if(i%3 == 0 || i%3 == 2) {
//                        temp_point.at(0) -= 100;
//                        run_point.push_back(temp_point);
//                        temp_point.clear();
//                    } else {
//                        run_point.push_back(temp_point);
//                    }
//                }
//                robot->m_getpointstate = true;
//            } else {
//                printf("获取穴位信息失败！\n");
//            }
//        } else {
//            run_point_number = size(run_point);
//            for(int i = 0; i < run_point_number; i++) {
//                temp_point = run_point.at(i);
//                for(int j = 0; j < 3; j++) {
//                    point[j] = temp_point.at(j);
//                }
//                robot->ProcessPosition(point, );
//            }
//        }
//    }
}


void RobotControl::ConnectRobot()
{
    if(!m_connectstate) {           //当前未连接
        m_arm = new XArmAPI("192.168.1.238");
        sleep_milliseconds(500);
        if(m_arm->error_code != 0) m_arm->clean_error();
        if(m_arm->warn_code != 0) m_arm->clean_warn();
        if(m_arm->is_connected()) {
            m_connectstate = true;
            printf("设备连接成功。\n");
        } else {
            if(m_arm) delete m_arm;
            printf("设备连接失败，请重新连接。\n");
        }
    } else {
        if(m_arm->error_code != 0) m_arm->clean_error();
        if(m_arm->warn_code != 0) m_arm->clean_warn();
        if(m_arm->is_connected()) {
            printf("设备已经连接。\n");
        } else {
            m_connectstate = false;
            if(m_arm) delete m_arm;
            printf("设备连接异常，请重新连接。\n");
        }
    }
}

void RobotControl::DisconnectRobot()
{
    if(m_arm->is_connected()) {
        EnableRobot(false);//关闭机器人
        m_arm->disconnect();//断开连接
        if(m_arm) delete m_arm;
        m_connectstate = false;
        printf("设备断开成功。\n");
    } else {
        if(m_arm) delete m_arm;
        printf("设备已经断开。\n");
    }
}

void RobotControl::EnableRobot(bool state)
{
    if(m_arm) {
        if(m_arm->is_connected()) {
            if (m_arm->error_code != 0) m_arm->clean_error();
            if (m_arm->warn_code != 0) m_arm->clean_warn();

            if(state) {
                m_arm->motion_enable(state);
                m_arm->set_mode(0);
                m_arm->set_state(0);
                m_enablestate = true;
                printf("机器人使能成功。\n");
            } else {
                m_enablestate = false;
                printf("机器人关闭成功。\n");
            }
        } else {
            m_connectstate = false;
            m_enablestate = false;
            printf("设备未连接，请重新连接。\n");
        }
    } else {
        m_connectstate = false;
        m_enablestate = false;
        printf("设备未连接，请重新连接。\n");
    }
}

void RobotControl::SetMassage(bool state)
{
    if(state == true) {
        m_massagestate = true;
        m_arm->set_gripper_enable(true);
        CreatThread();
    } else {
        m_massagestate = false;
    }
}

void RobotControl::SetTcpOffest(float tcp_offest[6])
{
    //xyz+rpy
    for(int i = 0; i < 6; i++) {
        m_tcpoffset[i] = tcp_offest[i];
    }
    //矩阵
    float offset_matrix[4][4] = {{0.0}};
    PoseToHomogenousMatrix4f(tcp_offest, offset_matrix);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            m_tcpoffsetmatrix[i][j] = offset_matrix[i][j];
        }
    }
}

void RobotControl::SetTcpOffsetFromFile(const string &filename)
{
//    const std::string filename = "/home/hua/IOROBOT/MassageRobot/TcpOffset.txt";
    FILE *fp = fopen(filename.c_str(),"r");
    bool flag = true;
    float temp[4] = {0.0};
    int row_number = 0;
    if(fp == NULL) {
        printf("文件打开失败。/n");
        return;
    } else {
        while (flag) {
            for(int i = 0; i < 4; i++) {
                if(EOF == fscanf(fp, "%f", &temp[i])) {
                    flag = false;
                    break;
                }
                if(i == 3) {
                    m_tcpoffsetmatrix[row_number][i] = temp[i]/1000.0;
                } else {
                    m_tcpoffsetmatrix[row_number][i] = temp[i];
                }
            }
            row_number++;
        }
    }
    fclose(fp);
}

void RobotControl::SetBaseOffest(float base_offest[])
{
    //xyz+rpy
    for(int i = 0; i < 6; i++) {
        m_baseoffset[i] = base_offest[i];
    }
    //矩阵
    float offset_matrix[4][4] = {{0.0}};
    PoseToHomogenousMatrix4f(base_offest, offset_matrix);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            m_baseoffsetmatrix[i][j] = offset_matrix[i][j];
        }
    }
}

void RobotControl::SetBaseOffsetFromFile(const std::string &filename)
{
    FILE *fp = fopen(filename.c_str(),"r");
    bool flag = true;
    float temp[4] = {0.0};
    int row_number = 0;
    if(fp == NULL) {
        printf("文件打开失败。/n");
        return;
    } else {
        while (flag) {
            for(int i = 0; i < 4; i++) {
                if(EOF == fscanf(fp, "%f", &temp[i])) {
                    flag = false;
                    break;
                }
                if(i == 3) {
                    m_baseoffsetmatrix[row_number][i] = temp[i]/1000.0;
                } else {
                    m_baseoffsetmatrix[row_number][i] = temp[i];
                }
            }
            row_number++;
        }
    }
    fclose(fp);
}

void RobotControl::ProcessPosition(float point_in_camera[3], vector<float> &run_point )
{
    //Step1：将工具相对于相机的位姿 变为 工具相对于基座的位姿
    float point_in_base[6] = {0.0,0.0,0.0,PI,0.0,0.0};
    float position_in_camera[4] = {0.0,0.0,0.0,1};
    float camera_in_base[4][4];         //相机在基座下
    float tool_in_base_f4[4] = {0.0};   //基坐标系下,工具的位置
    float tool_in_base_f3[3] = {0.0};   //基坐标系下,工具的位置
    float tool_in_end_f3[3]  = {0.0};   //末端坐标系下,工具的位置
    float rotation_tool_in_base[3][3];  //基坐标系下,工具的姿态矩阵
    //穴位在相机下的位置
    for(int i = 0; i < 3; i++) {
        position_in_camera[i] = point_in_camera[i]/1000.0;
    }
    IdentityMatrix(camera_in_base);
    //相机在基座下的位置
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            camera_in_base[i][j] = m_baseoffsetmatrix[i][j];
        }
    }
    //工具在基坐标系下的位置
    MatrixMultiplyVector4f(camera_in_base, position_in_camera, tool_in_base_f4);
    //Step2：将工具相对于基座的位姿 变为 末端相对于基座的位姿
    float matrix_tool_in_base[4][4];
    //工具在基坐标系下的齐次矩阵
    PoseToHomogenousMatrix4f(point_in_base, matrix_tool_in_base);
    for(int i = 0; i < 3; i++) {
        tool_in_base_f3[i]  = tool_in_base_f4[i];
        tool_in_end_f3[i]   = m_tcpoffsetmatrix[i][3];
        for(int j = 0; j < 3; j++) {
            rotation_tool_in_base[i][j] = matrix_tool_in_base[i][j];
        }
    }
    float temp_vector[3] = {0.0};
    MatrixMultiplyVector3f(rotation_tool_in_base, tool_in_end_f3, temp_vector);
    for(int i = 0; i < 3; i++) {
        point_in_base[i] = (tool_in_base_f3[i] - temp_vector[i]) * 1000.0;
    }
    point_in_base[3] = 180.0;
    for(int i = 0; i < 6; i++) {
        run_point.push_back(point_in_base[i]);
    }
//    m_arm->set_position(point_in_base, true);
}

int RobotControl::ReadPointFromTXT(const std::string &filename, std::vector<std::vector<float> > &point_vector)
{
    FILE *fp = fopen(filename.c_str(),"r");
    bool flag = true;
    vector<float> point(3);
    float acupoint[3] = {0.0};
    if(fp == NULL) {
        printf("文件打开失败。/n");
        return 0;
    } else {
        while (flag) {
            for(int i = 0; i < 3; i++) {
                if(EOF == fscanf(fp, "%f", &acupoint[i])) {
                    flag = false;
                    break;
                }
                point.at(i) = acupoint[i];
            }
            if(flag) point_vector.push_back(point);
        }
    }
    fclose(fp);
    return 1;
}

void RobotControl::PoseToHomogenousMatrix4f(float pose[6], float matrix[4][4])
{
    IdentityMatrix(matrix);
    //rpy:分别绕X,Y,Z轴旋转gama,beta,alpha
    float gama = 0.0, beta = 0.0, alpha = 0.0;
    gama  = pose[3];
    beta  = pose[4];
    alpha = pose[5];
    matrix[0][0] = cos(alpha) * cos(beta);
    matrix[0][1] = cos(alpha) * sin(beta) * sin(gama) - sin(alpha) * cos(gama);
    matrix[0][2] = cos(alpha) * sin(beta) * cos(gama) + sin(alpha) * sin(gama);
    matrix[0][3] = pose[0];
    matrix[1][0] = sin(alpha) * cos(beta);
    matrix[1][1] = sin(alpha) * sin(beta) * sin(gama) + cos(alpha) * cos(gama);
    matrix[1][2] = sin(alpha) * sin(beta) * cos(gama) - cos(alpha) * sin(gama);
    matrix[1][3] = pose[1];
    matrix[2][0] = -sin(beta);
    matrix[2][1] = cos(beta) * sin(gama);
    matrix[2][2] = cos(beta) * cos(gama);
    matrix[2][3] = pose[2];
}

void RobotControl::IdentityMatrix(float matrix[4][4])
{
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            if(i == j) {
                matrix[i][j] = 1;
            } else {
                matrix[i][j] = 0;
            }
        }
    }
}

void RobotControl::InverseMatrix(float matrix[4][4], float inverse_matrix[4][4])
{
    float rotation_matrix[3][3] = {{0.0}};
    float rotation_matrix_transpose[3][3] = {{0.0}};
    float position[3] = {0.0};
    for(int i = 0; i < 3; i++) {
        position[i] = matrix[i][3];                 //位置
        for(int j = 0; j < 3; j++) {
            rotation_matrix[i][j] = matrix[i][j];    //姿态
        }
    }
    TransposeMatrix(rotation_matrix, rotation_matrix_transpose);       //姿态转置
    IdentityMatrix(inverse_matrix);
    for(int i = 0; i < 3; i++) {
        inverse_matrix[i][3] = - (rotation_matrix_transpose[i][0]*position[0] + rotation_matrix_transpose[i][1]*position[1]
                + rotation_matrix_transpose[i][2]*position[2]);         //位置
        for(int j = 0; j < 3; j++) {
            //            inverse_matrix[i][3] -= rotation_matrix_transpose[i][j]*position[j];      //位置
            inverse_matrix[i][j] = rotation_matrix_transpose[i][j];    //姿态
        }
    }
}

void RobotControl::TransposeMatrix(float matrix[3][3], float transpose_matrix[3][3])
{
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            transpose_matrix[i][j] = matrix[j][i];
        }
    }
}

void RobotControl::MatrixMultiplyVector4f(float matrix[4][4], float vector[4], float result[4])
{
    for(int i = 0; i < 4; i++) {
        result[i] = matrix[i][0] * vector[0] + matrix[i][1] * vector[1] + matrix[i][2] * vector[2] + matrix[i][3] * vector[3];
//        for(int j = 0; j < 4; j++) {
//            result[i] += matrix[i][j] * vector[j];
//        }
    }
}

void RobotControl::MatrixMultiplyVector3f(float matrix[3][3], float vector[3], float result[3])
{
    for(int i = 0; i < 3; i++) {
        result[i] = matrix[i][0] * vector[0] + matrix[i][1] * vector[1] + matrix[i][2] * vector[2];
//        for(int j = 0; j < 3; j++) {
//            result[i] += matrix[i][j] * vector[j];
//        }
    }
}
