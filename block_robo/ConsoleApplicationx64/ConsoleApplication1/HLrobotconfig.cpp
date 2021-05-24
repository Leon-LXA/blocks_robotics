#include "./eigen3/Eigen/Dense"
#include <iostream>
#include "HLrobotconfig.h"

double PI = 3.1415926;
using namespace std;
using namespace Eigen;
//arm length
double L1 = 491;
double L2 = 450;
double L3 = 450;
double L4 = 84;
// joint angle limits
// theta1 = -170~170;
// theta2 = -120~120;
// theta3 = -228~48;
// theta4 = -170~170;
// theta5 = -120~120;
// theta6 = -360~360;
// forward kinematics
// test1 (-27.84,2.947,78.849,-0.033,97.072,147.849)
// test2 (-5.582,14.641,65.347,-0.054,98.88,148.946)
// test3 (29.27,-2.795,89.227,-0.691,92.67,183.769)
// inverse kinematics
// test1 (415.746,-219.622,920.636,-29.489,178.867,-33.751)
// test2 (555.893,-54.405,920.636,-8.288,178.867,-33.75)
// test3 (374.286,208.621,884.491,-8.287,178.867,-33.75)

namespace HLRobot
{

    //init TransMatrix
    double mTransMatrix[16]{ 0 };
    //use one cfg only
    bool mConfig[3] = { 1, 1, 1 };

    Matrix3d OmegaHat(Vector3d omega)
    {
        Matrix3d omega_hat;
        omega_hat << 0, -omega(2), omega(1),
            omega(2), 0, -omega(0),
            -omega(1), omega(0), 0;
        return omega_hat;
    }

    //calculate exp mat of omega_hat
    Matrix3d Exponential_3(Vector3d omega, double theta)
    {
        Matrix3d omega_hat;
        Matrix3d e_omega_theta;
        Matrix3d E; //eye
        E << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

        omega_hat = HLRobot::OmegaHat(omega);
        double omega_module = sqrt(pow(omega(0), 2) + pow(omega(1), 2) + pow(omega(2), 2));
        e_omega_theta = E + omega_hat * sin(omega_module * theta * PI / 180) / omega_module + omega_hat * omega_hat * (1 - cos(omega_module * theta * PI / 180)) / pow(omega_module, 2);

        //cout<<"omega_hat="<<omega_hat<<endl;
        //cout<<"omega_hat*sin(omega_module*theta*180/PI)="<<omega_hat*sin(omega_module*theta*180/PI)<<endl;
        //cout<<"omega_hat*omega_hat*(1-cos(omega_module*theta*PI/180))/pow(omega_module,2)="<<omega_hat*omega_hat*(1-cos(omega_module*theta*PI/180))/pow(omega_module,2)<<endl;
        //cout<<e_omega_theta<<endl<<endl;

        return e_omega_theta;
    }

    //calculate exp mat of xi_hat
    Matrix4d Exponential_4(Vector3d v, Vector3d w, double theta)
    {
        Vector3d p;
        Matrix4d e_xi_theta;
        Matrix3d R;
        Matrix3d E; //eye
        E << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

        R = HLRobot::Exponential_3(w, theta);

        p = (E - R) * (w.cross(v)) + w * w.transpose() * v * (theta * PI / 180);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                e_xi_theta(i, j) = R(i, j);
            }
            e_xi_theta(i, 3) = p(i);
        }

        e_xi_theta(3, 0) = 0;
        e_xi_theta(3, 1) = 0;
        e_xi_theta(3, 2) = 0;
        e_xi_theta(3, 3) = 1;

        //cout<<e_xi_theta<<endl<<endl;
        return e_xi_theta;
    }

    double SubProblem1(Vector3d p, Vector3d q, Vector3d r, Vector3d w)
    {
        //pksub1，single axis rotation
        //in：p(start)，q(end)，r(pt on axis)，xi(axis vec)
        //out：theta(rotation angle)
        Vector3d u = p - r;
        Vector3d v = q - r;
        Vector3d u_1; //u' projection of u
        Vector3d v_1; //v'

        double theta = 0;
        u_1 = u - w * w.transpose() * u;
        v_1 = v - w * w.transpose() * v;

        if (u(0) == 0 && u(1) == 0 && u(2) == 0)
            theta = 0;
        else
            theta = atan2(w.transpose() * (u_1.cross(v_1)), u_1.transpose() * v_1);
        if (theta < -PI / 2)
        {
            theta += 2 * PI;
        }

        return theta;
    }

    void SubProblem2(Vector3d p, Vector3d q, Vector3d r, Vector3d w2, Vector3d w1, double& theta2, double& theta1)
    {
        //pksub2，rotation about 2 intersecting axis
        //in：p(start)，q(end)，r(intersection pt)，xi2(first axis)，xi1(second axis)
        //out：theta2(first rotation)，theta1(second rotation)
        Vector3d u = p - r;
        Vector3d v = q - r;
        double alpha, beta, gama;
        double alpha1 = (w1.transpose() * w2) * (w2.transpose()) * u;
        double alpha2 = -w1.transpose() * v;

        alpha = (alpha1 + alpha2) / (pow((w1.transpose() * w2), 2) - 1);
        double beta1 = (w1.transpose() * w2) * (w1.transpose()) * v;
        double beta2 = -w2.transpose() * u;

        beta = (beta1 + beta2) / (pow((w1.transpose() * w2), 2) - 1);

        gama = pow(((u.transpose() * u - pow(alpha, 2) - pow(beta, 2) - 2 * alpha * beta * w1.transpose() * w2) / (pow((w1.cross(w2)).norm(), 2))), 0.5);

        Vector3d z = alpha * w1 + beta * w2 - gama * (w1.cross(w2));
        Vector3d c = z + r;

        Vector3d z2 = alpha * w1 + beta * w2 + gama * (w1.cross(w2));
        Vector3d c2 = z2 + r;

        double theta2_t = HLRobot::SubProblem1(p, c, r, w2);
        double theta1_t = HLRobot::SubProblem1(c, q, r, w1);

        double theta3_t = HLRobot::SubProblem1(c2, q, r, w1);
        double theta4_t = HLRobot::SubProblem1(p, c2, r, w2);

        if (abs(theta1_t) + abs(theta2_t) < abs(theta3_t) + abs(theta4_t))
        {
            theta1 = theta1_t;
            theta2 = theta2_t;
        }
        else
        {
            theta1 = theta3_t;
            theta2 = theta4_t;
        }

    }
    double SubProblem3(Vector3d p, Vector3d q, Vector3d r, Vector3d w, double delta)
    {
        //pksub3，rotate to a given distance
        //in：p(start)，q(end)，r(pt on axis)，xi(axis vec)，d(distance)
        //out：theta(angle)
        Vector3d u = p - r;
        Vector3d v = q - r;
        Vector3d u_1 = u - w * w.transpose() * u;
        Vector3d v_1 = v - w * w.transpose() * v;
        double delta_1 = pow((pow(delta, 2) - pow((w.transpose() * (p - q)).norm(), 2)), 0.5);
        double theta0 = atan2(w.transpose() * (u_1.cross(v_1)), u_1.transpose() * v_1);

        // cout << theta0 << endl;
        // cout << delta_1 << endl;
        // cout << u_1.norm() << endl;
        // cout << v_1.norm() << endl;

        double theta;
        if (u_1.norm() * v_1.norm() < 0.00000000001)
        {
            theta = 0;
        }
        else
        {
            theta = theta0 - acos((pow(u_1.norm(), 2) + pow(v_1.norm(), 2) - pow(delta_1, 2)) / (2 * u_1.norm() * v_1.norm()));
        }
        return theta;
    }

    //原给定函数
    void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
    {
        Vector3d X(1, 0, 0);
        Vector3d Y(0, 1, 0);
        Vector3d Z(0, 0, 1);

        Matrix3d R_x;
        Matrix3d R_y;
        Matrix3d R_z;

        R_x = HLRobot::Exponential_3(Z, yaw);
        R_y = HLRobot::Exponential_3(Y, pitch);
        R_z = HLRobot::Exponential_3(Z, roll);
        Matrix3d R_ab = R_x * R_y * R_z;

        Matrix4d G_ab;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                G_ab(i, j) = R_ab(i, j);
            }
        }
        G_ab(0, 3) = x;
        G_ab(1, 3) = y;
        G_ab(2, 3) = z;
        G_ab(3, 0) = 0;
        G_ab(3, 1) = 0;
        G_ab(3, 2) = 0;
        G_ab(3, 3) = 1;

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                mTransMatrix[4 * i + j] = G_ab(i, j);
            }
        }
    }

    //逆运动学获得轴角度
    void GetJointAngles(double& angle1, double& angle2, double& angle3, double& angle4, double& angle5, double& angle6)
    {
        double theta1;
        double theta2;
        double theta3;
        double theta4;
        double theta5;
        double theta6;

        Vector3d X(1, 0, 0);
        Vector3d Y(0, 1, 0);
        Vector3d Z(0, 0, 1);

        Matrix4d gst0;
        gst0 << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, (L1 + L2 + L3 + L4),
            0, 0, 0, 1;

        Matrix4d gd; //期望位型
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                gd(i, j) = mTransMatrix[4 * i + j];
            }
        }
        Matrix4d g1 = gd * gst0.inverse();

        Vector3d w1(0, 0, 1);
        Vector3d w2(0, 1, 0);
        Vector3d w3(0, 1, 0);
        Vector3d w4(0, 0, 1);
        Vector3d w5(0, 1, 0);
        Vector3d w6(0, 0, 1);

        Vector3d q1(0, 0, 0);
        Vector3d q2(0, 0, L1);
        Vector3d q3(0, 0, L1 + L2);
        Vector3d q4(0, 0, 0);
        Vector3d q5(0, 0, L1 + L2 + L3);
        Vector3d q6(0, 0, 0);

        Vector3d v1 = q1.cross(w1);
        Vector3d v2 = q2.cross(w2);
        Vector3d v3 = q3.cross(w3);
        Vector3d v4 = q4.cross(w4);
        Vector3d v5 = q5.cross(w5);
        Vector3d v6 = q6.cross(w6);

        Vector3d P1(0, 0, L1 + L2 + L3);
        Vector3d P2(0, 0, L1);
        Vector3d P3(0, 0, L1);
        Vector3d P4(0, 10, 0);

        Matrix4d exi1;
        Matrix4d exi2;
        Matrix4d exi3;
        Matrix4d exi4;
        Matrix4d exi5;
        Matrix4d exi6;

        //pksub3 -> theta3
        Vector4d P01(P1(0), P1(1), P1(2), 1);
        Vector4d P02(P2(0), P2(1), P2(2), 1);
        double d1 = (g1 * P01 - P02).norm();
        theta3 = HLRobot::SubProblem3(P1, P2, q3, w3, d1);

        //pksub2 -> theta1，theta2
        exi3 = HLRobot::Exponential_4(v3, w3, theta3 * 180 / PI);
        Vector4d P11(P1(0), P1(1), P1(2), 1);
        Vector4d P12 = exi3 * P11;
        Vector3d P13(P12(0), P12(1), P12(2));
        Vector4d Q11 = g1 * P11;
        Vector3d Q12(Q11(0), Q11(1), Q11(2));
        SubProblem2(P13, Q12, P2, w2, w1, theta2, theta1);

        //pksub2 -> theta4,theta5
        exi1 = HLRobot::Exponential_4(v1, w1, theta1 * 180 / PI);
        exi2 = HLRobot::Exponential_4(v2, w2, theta2 * 180 / PI);
        Vector4d P21(P3(0), P3(1), P3(2), 1);
        Matrix4d g2 = ((exi1 * exi2 * exi3).inverse()) * g1;
        Vector4d Q21 = g2 * P21;
        Vector3d Q22(Q21(0), Q21(1), Q21(2));
        HLRobot::SubProblem2(P3, Q22, P1, w5, w4, theta5, theta4);
        //theta5 = PI - theta5;

        //pksub1 -> theta6
        exi4 = HLRobot::Exponential_4(v4, w4, theta4 * 180 / PI);
        exi5 = HLRobot::Exponential_4(v5, w5, theta5 * 180 / PI);
        Vector4d P31(P4(0), P4(1), P4(2), 1);
        Matrix4d g3 = ((exi4 * exi5).inverse()) * g2;
        Vector4d Q31 = g3 * P31;

        // (-5.582,14.641,65.347,-0.054,98.88,148.946)
        // theta1=-5.582*PI/180;
        // theta2=14.641*PI/180;
        // theta3=65.347*PI/180;
        // theta4=-0.054*PI/180;
        // theta5=98.88*PI/180;
        // exi1=HLRobot::Exponential_4(v1, w1,theta1*180/PI);
        // exi2=HLRobot::Exponential_4(v2, w2,theta2*180/PI);
        // exi3=HLRobot::Exponential_4(v3, w3,theta3*180/PI);
        // exi4=HLRobot::Exponential_4(v4, w4,theta4*180/PI);
        // exi5=HLRobot::Exponential_4(v5, w5,theta5*180/PI);
        // g2 =( (exi1 * exi2 * exi3).inverse()) * g1;
        // g3 = ((exi4 * exi5).inverse()) * g2;
        // Q31 = g3 * P31;

        Vector3d Q32(Q31(0), Q31(1), Q31(2));

        theta6 = SubProblem1(P4, Q32, q6, w6);

        angle1 = theta1 * 180 / PI;
        angle2 = theta2 * 180 / PI;
        angle3 = theta3 * 180 / PI;
        angle4 = theta4 * 180 / PI;
        angle5 = theta5 * 180 / PI;
        angle6 = theta6 * 180 / PI;

        if (angle1 > 170)
            angle1 -= 360;
        if (angle1 < -170)
            angle1 += 360;
        if (angle2 > 120)
            angle2 -= 360;
        if (angle2 < -120)
            angle2 += 360;
        if (angle3 > 136)
            angle3 -= 360;
        //cout << angle3 << endl;
        if (angle3 < -136)
            angle3 += 360;
        if (angle4 > 185)
            angle4 -= 360;
        if (angle4 < -185)
            angle4 += 360;
        if (angle5 > 120)
            angle5 -= 360;
        if (angle5 < -120)
            angle5 += 360;
        //angle5 += 180;
        if (angle6 > 360)
            angle6 -= 360;
        if (angle6 < -360)
            angle6 += 360;
    }

    void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6)
    {
        double theta1 = angle1; //*PI/180;
        double theta2 = angle2; //*PI/180;
        double theta3 = angle3; //*PI/180;
        double theta4 = angle4; //*PI/180;
        double theta5 = angle5; //*PI/180;
        double theta6 = angle6; //*PI/180;

        Vector3d omega1(0, 0, 1);
        Vector3d omega2(0, 1, 0);
        Vector3d omega3(0, 1, 0);
        Vector3d omega4(0, 0, 1);
        Vector3d omega5(0, 1, 0);
        Vector3d omega6(0, 0, 1);

        Vector3d q1(0, 0, 0);
        Vector3d q2(0, 0, L1);
        Vector3d q3(0, 0, L1 + L2);
        Vector3d q4(0, 0, 0);
        Vector3d q5(0, 0, L1 + L2 + L3);
        Vector3d q6(0, 0, 0);

        Vector3d v1 = -HLRobot::OmegaHat(omega1) * q1;
        Vector3d v2 = -HLRobot::OmegaHat(omega2) * q2;
        Vector3d v3 = -HLRobot::OmegaHat(omega3) * q3;
        Vector3d v4 = -HLRobot::OmegaHat(omega4) * q4;
        Vector3d v5 = -HLRobot::OmegaHat(omega5) * q5;
        Vector3d v6 = -HLRobot::OmegaHat(omega6) * q6;

        Matrix4d gst0;
        gst0 << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, (L1 + L2 + L3 + L4),
            0, 0, 0, 1;
        Matrix4d gst1 = Exponential_4(v1, omega1, theta1);
        Matrix4d gst2 = Exponential_4(v2, omega2, theta2);
        Matrix4d gst3 = Exponential_4(v3, omega3, theta3);
        Matrix4d gst4 = Exponential_4(v4, omega4, theta4);
        Matrix4d gst5 = Exponential_4(v5, omega5, theta5);
        Matrix4d gst6 = Exponential_4(v6, omega6, theta6);

        Matrix4d gst = gst1 * gst2 * gst3 * gst4 * gst5 * gst6 * gst0;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                mTransMatrix[4 * i + j] = gst(i, j);
            }
        }
    }

    void GetJointEndPos(double& x, double& y, double& z, double& yaw, double& pitch, double& roll)
    {
        Matrix4d gst;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                gst(i, j) = mTransMatrix[4 * i + j];
            }
        }

        x = gst(0, 3);
        y = gst(1, 3);
        z = gst(2, 3);
        double alpha;
        double beta;
        double gamma;

        beta = atan2(sqrt(pow(gst(2, 0), 2) + pow(gst(2, 1), 2)), gst(2, 2));
        alpha = atan2(gst(1, 2) / sin(beta), gst(0, 2) / sin(beta));
        gamma = atan2(gst(2, 1) / sin(beta), -gst(2, 0) / sin(beta));
        yaw = alpha * 180 / PI;
        pitch = beta * 180 / PI;
        roll = gamma * 180 / PI - 180;

        if (roll > 90)
            roll = roll - 180;
        else if (roll < -90)
            roll = roll + 180;

        cout << "x=" << x << endl;
        cout << "y=" << y << endl;
        cout << "z=" << z << endl;
        cout << "yaw=" << yaw << endl;
        cout << "pitch=" << pitch << endl;
        cout << "roll=" << roll << endl;
    }

    // /********************************************************************
    // ABSTRACT:	机器人逆运动学

    // INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

    //             config[3]：姿态，六轴机器人对应有8种姿态 (即对应的逆运动学8个解)，为了安全，
    //             实验室中我们只计算一种即可。config用来作为选解的标志数。

    // OUTPUTS:    theta[6] 6个关节角, 单位为弧度

    // RETURN:		<none>
    // ***********************************************************************/
    // void robotBackward(const double* TransVector, bool* mconfig, double* theta)
    // {

    // }

    // /********************************************************************
    // ABSTRACT:	机器人正运动学

    // INPUTS:		q[6]: 6个关节角, 单位为弧度

    // OUTPUTS:	config[3]：姿态，六轴机器人对应有8种姿态，为了安全，
    //             实验室中我们只计算一种即可。config用来作为选解的标志数。

    //             TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米

    // RETURN:		<none>
    // ***********************************************************************/
    // void robotForward(const double* q, double* TransVector, bool* mconfig)
    // {

    // }
}
