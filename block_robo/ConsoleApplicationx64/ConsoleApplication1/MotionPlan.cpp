#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "HLrobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include "eigen3/Eigen/Dense"

#define PI 3.14159
#define singleT 3;

using namespace std;
using namespace HLRobot;
using namespace Eigen;

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

CHLMotionPlan::CHLMotionPlan()
{
	for (int i = 0; i < 6; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}

	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;
	td[0] = singleT;
	td[1] = singleT;
	td[2] = 2 * singleT;
	td[3] = singleT;
	td[4] = singleT;
	td[5] = singleT;
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{

}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.003)
	{
		mSampleTime = 0.003;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double vel, double acc, double dec)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和技术点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints(PosStruct startPos, PosStruct endPos)
{
	double startAngle[3], endAngle[3];

	startAngle[0] = startPos.yaw * PI / 180;
	startAngle[1] = startPos.pitch * PI / 180;
	startAngle[2] = startPos.roll * PI / 180;

	endAngle[0] = endPos.yaw * PI / 180;
	endAngle[1] = endPos.pitch * PI / 180;
	endAngle[2] = endPos.roll * PI / 180;

	start_yaw = startPos.yaw;
	start_pitch = startPos.pitch;
	start_roll = startPos.roll;

	end_yaw = endPos.yaw;
	end_pitch = endPos.pitch;
	end_roll = endPos.roll;

	mStartMatrixData[0] = cos(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) - sin(startAngle[0])*sin(startAngle[2]);
	mStartMatrixData[1] = -cos(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) - sin(startAngle[0])*cos(startAngle[2]);
	mStartMatrixData[2] = cos(startAngle[0])*sin(startAngle[1]);
	mStartMatrixData[3] = startPos.x / 1000;

	mStartMatrixData[4] = sin(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) + cos(startAngle[0])*sin(startAngle[2]);
	mStartMatrixData[5] = -sin(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) + cos(startAngle[0])*cos(startAngle[2]);
	mStartMatrixData[6] = sin(startAngle[0])*sin(startAngle[1]);
	mStartMatrixData[7] = startPos.y / 1000;

	mStartMatrixData[8] = -sin(startAngle[1])*cos(startAngle[2]);
	mStartMatrixData[9] = sin(startAngle[1])*sin(startAngle[2]);
	mStartMatrixData[10] = cos(startAngle[1]);
	mStartMatrixData[11] = startPos.z / 1000;

	mStartMatrixData[12] = 0;
	mStartMatrixData[13] = 0;
	mStartMatrixData[14] = 0;
	mStartMatrixData[15] = 1;

	mEndMatrixData[0] = cos(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) - sin(endAngle[0])*sin(endAngle[2]);
	mEndMatrixData[1] = -cos(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) - sin(endAngle[0])*cos(endAngle[2]);
	mEndMatrixData[2] = cos(endAngle[0])*sin(endAngle[1]);
	mEndMatrixData[3] = endPos.x / 1000;

	mEndMatrixData[4] = sin(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) + cos(endAngle[0])*sin(endAngle[2]);
	mEndMatrixData[5] = -sin(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) + cos(endAngle[0])*cos(endAngle[2]);
	mEndMatrixData[6] = sin(endAngle[0])*sin(endAngle[1]);
	mEndMatrixData[7] = endPos.y / 1000;

	mEndMatrixData[8] = -sin(endAngle[1])*cos(endAngle[2]);
	mEndMatrixData[9] = sin(endAngle[1])*sin(endAngle[2]);
	mEndMatrixData[10] = cos(endAngle[1]);
	mEndMatrixData[11] = endPos.z / 1000;

	mEndMatrixData[12] = 0;
	mEndMatrixData[13] = 0;
	mEndMatrixData[14] = 0;
	mEndMatrixData[15] = 1;

	double angle1, angle2, angle3, angle4, angle5, angle6;
	HLRobot::SetRobotEndPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);
	
	mJointAngleBegin[0] = angle1;
	mJointAngleBegin[1] = angle2;
	mJointAngleBegin[2] = angle3;
	mJointAngleBegin[3] = angle4;
	mJointAngleBegin[4] = angle5;
	mJointAngleBegin[5] = angle6;

	

	HLRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);
	mJointAngleEnd[0] = angle1;
	mJointAngleEnd[1] = angle2;
	mJointAngleEnd[2] = angle3;
	mJointAngleEnd[3] = angle4;
	mJointAngleEnd[4] = angle5;
	mJointAngleEnd[5] = angle6;

	/*for (int i = 0; i < 6; i++)
	{
		cout << mJointAngleBegin[i] << endl;
	}
	cout << endl;
	for (int i = 0; i < 6; i++)
	{
		cout << mJointAngleEnd[i] << endl;
	}*/

}

/********************************************************************
ABSTRACT:	运动轨迹规划部分（以关节空间为例）

INPUTS:		pos						二维位置向量

OUTPUTS:	pos						二维位置向量（第一维：位置个数，第二维：每个轴的关节角度，单位弧度）

RETURN:		<none>
***********************************************************************/

/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
//暂时用不到关节空间规划
//void CHLMotionPlan::GetPlanPoints()//暂时用不到关节空间规划
//{
//	ofstream outfile;               			//创建文件
//	outfile.open("./mydata.txt");
//	/*outfile << mJointAngleBegin[0] << "  "
//			<< mJointAngleBegin[1] << "  "
//			<< mJointAngleBegin[2] << "  "
//			<< mJointAngleBegin[3] << "  "
//			<< mJointAngleBegin[4] << "  "
//			<< mJointAngleBegin[5] << "  ";
//	outfile << endl;//保存初始的时间、六个关节角度
//	*/
//
//	//完成代码
//
//	//通过规划得到每一段的曲线参数
//	LFPB_Planning(mJointAngleBegin[0], mJointAngleEnd[0], 0);
//	LFPB_Planning(mJointAngleBegin[1], mJointAngleEnd[1], 1);
//	LFPB_Planning(mJointAngleBegin[2], mJointAngleEnd[2], 2);
//	LFPB_Planning(mJointAngleBegin[3], mJointAngleEnd[3], 3);
//	LFPB_Planning(mJointAngleBegin[4], mJointAngleEnd[4], 4);
//	LFPB_Planning(mJointAngleBegin[5], mJointAngleEnd[5], 5);
//	//cout << mSampleTime << endl;
//	//double length = td / mSampleTime;
//	//cout << length << endl;
//	//cout <<"tb= " <<tb[0] << endl << tb[1] <<endl<< tb[2] <<endl<< tb[3] <<endl<< tb[4]<<endl << tb[5];
//	for (int i = 0; i < td/mSampleTime; i++)
//	{
//		//cout <<" here !" << endl;
//			PosStruct temp;
//			double t = mSampleTime * i;
//
//			if (t < tb[0])
//				temp.x = 0.5 * a[0] * t * t + mJointAngleBegin[0];
//			else if (t >= tb[0] && t < (tb[0] + to[0]))
//				temp.x = 0.5 * a[0] * tb[0] * tb[0] + (t - tb[0]) * v[0] + mJointAngleBegin[0];
//			else
//				temp.x = 0.5 * a[0] * tb[0] * tb[0] + to[0] * v[0] + v[0] * (t - tb[0] - to[0]) - 0.5 * a[0] * (t - tb[0] - to[0]) * (t - tb[0] - to[0]) + mJointAngleBegin[0];
//			
//			if (t < tb[1])
//				temp.y = 0.5 * a[1] * t * t + mJointAngleBegin[1];
//			else if (t >= tb[1] && t < (tb[1] + to[1]))
//				temp.y = 0.5 * a[1] * tb[1] * tb[1] + (t - tb[1]) * v[1] + mJointAngleBegin[1];
//			else
//				temp.y = 0.5 * a[1] * tb[1] * tb[1] + to[1] * v[1] + v[1] * (t - tb[1] - to[1]) - 0.5 * a[1] * (t - tb[1] - to[1]) * (t - tb[1] - to[1]) + mJointAngleBegin[1];
//			
//			if (t < tb[2])
//				temp.z = 0.5 * a[2] * t * t + mJointAngleBegin[2];
//			else if (t >= tb[2] && t < (tb[2] + to[2]))
//				temp.z = 0.5 * a[2] * tb[2] * tb[2] + (t - tb[2]) * v[2] + mJointAngleBegin[2];
//			else
//				temp.z = 0.5 * a[2] * tb[2] * tb[2] + to[2] * v[2] + v[2] * (t - tb[2] - to[2]) - 0.5 * a[2] * (t - tb[2] - to[2]) * (t - tb[2] - to[2]) + mJointAngleBegin[2];
//
//			if (t < tb[3])
//				temp.yaw = 0.5 * a[3] * t * t + mJointAngleBegin[3];
//			else if (t >= tb[3] && t < (tb[3] + to[3]))
//				temp.yaw = 0.5 * a[3] * tb[3] * tb[3] + (t - tb[3]) * v[3] + mJointAngleBegin[3];
//			else
//				temp.yaw = 0.5 * a[3] * tb[3] * tb[3] + to[3] * v[3] + v[3] * (t - tb[3] - to[3]) - 0.5 * a[3] * (t - tb[3] - to[3]) * (t - tb[3] - to[3]) + mJointAngleBegin[3];
//
//			if (t < tb[4])
//				temp.pitch = 0.5 * a[4] * t * t + mJointAngleBegin[4];
//			else if (t >= tb[4] && t < (tb[4] + to[4]))
//				temp.pitch = 0.5 * a[4] * tb[4] * tb[4] + (t - tb[4]) * v[4] + mJointAngleBegin[4];
//			else
//				temp.pitch = 0.5 * a[4] * tb[4] * tb[4] + to[4] * v[4] + v[4] * (t - tb[4] - to[4]) - 0.5 * a[4] * (t - tb[4] - to[4]) * (t - tb[4] - to[4]) + mJointAngleBegin[4];
//
//			if (t < tb[5])
//				temp.roll = 0.5 * a[5] * t * t + mJointAngleBegin[5];
//			else if (t >= tb[5] && t < (tb[5] + to[5]))
//				temp.roll = 0.5 * a[5] * tb[5] * tb[5] + (t - tb[5]) * v[5] + mJointAngleBegin[5];
//			else
//				temp.roll = 0.5 * a[5] * tb[5] * tb[5] + to[5] * v[5] + v[5] * (t - tb[5] - to[5]) - 0.5 * a[5] * (t - tb[5] - to[5]) * (t - tb[5] - to[5]) + mJointAngleBegin[5];
//
//			//cout << temp.x <<" "<<temp.y<<" "<<temp.z<<" "<< temp.yaw<<" "<<temp.pitch<<" "<< temp.roll<< endl;
//			outfile << temp.x << "  "
//				<< temp.y << "  "
//				<< temp.z << "  "
//				<< temp.yaw << "  "
//				<< temp.pitch << "  "
//				<< temp.roll << "  " << endl;
//			Point.push_back(temp);
//	}
//	outfile << endl;//保存所有数据
//
//}//暂时用不到这个关节空间规划

//void CHLMotionPlan::LFPB_Planning(double pos_start, double pos_end, int index)
//{
//	//先创建一个中间点，在笛卡尔坐标系的x，y方向直接取中值，z轴直接给一个固定值
//	double pos_waypoint;
//	if (index != 2)
//		pos_waypoint = 0.5 * (pos_start + pos_end);
//	else
//		pos_waypoint = heightWaypoint;
//	//First Segment
//	if (pos_start > pos_waypoint)
//		as[index] = -mAcc * 1000;
//	else
//		as[index] = mAcc * 1000;
//	
//	if (index == 2)
//		as[index] /= 10;
//
//	tb0[index] = td - sqrt(td * td -  2*(pos_waypoint - pos_start) / as[index]); //第一段的加速时间，我们预设两端轨迹时间相等
//	//cout <<"tb:" <<tb[index] << endl;
//	v01[index] = as[index] * tb0[index]; //线性运动速度
//	
//	//Last Segment
//	if (pos_end > pos_waypoint)
//		ae[index] = -mAcc * 1000;
//	else
//		ae[index] = mAcc * 1000;
//
//	if (index == 2)
//		ae[index] /= 10;
//
//	tb2[index] = td - sqrt(td * td + 2 * (pos_end - pos_waypoint) / ae[index]); //第一段的加速时间，我们预设两端轨迹时间相等
//	//cout <<"tb:" <<tb[index] << endl;
//	v12[index] = - ae[index] * tb2[index]; //线性运动速度
//
//	//Middle Blend
//	if (v12[index] > v01[index])
//		aw[index] = mAcc * 1000;
//	else
//		aw[index] = -mAcc * 1000;
//
//	if (index == 2)
//		aw[index] /= 20;
//
//	tb1[index] = (v12[index]-v01[index])/aw[index];
//
//	//把第一段和第二段的线性时间算出来
//	t01[index] = td - tb0[index] - 0.5 * tb1[index];
//	t12[index] = td - tb2[index] - 0.5 * tb1[index];
//
//	//cout << "LFPB" << endl;
//
//}
void CHLMotionPlan::LFPB_Planning(double pos_start, double pos_end, int index)
{
	//先创建一个中间点，在笛卡尔坐标系的x，y方向直接取中值，z轴直接给一个固定值
	double pos_waypoint;
	
	if (index != 2)
		pos_waypoint = 0.5 * (pos_start + pos_end);
	else
		pos_waypoint = heightWaypoint;
	//仅针对z轴，存在爬升阶段与降落阶段，因此多运动两段时间,只需要把总时间弄长就行
	//First Segment
	if (pos_start > pos_waypoint)
		as[index] = -mAcc * 1000;
	else
		as[index] = mAcc * 1000;

	if (index == 2)
		as[index] /= 20;

	tb0[index] = td[index] - sqrt(td[index] * td[index] - 2 * (pos_waypoint - pos_start) / as[index]); //第一段的加速时间，我们预设两端轨迹时间相等
	//cout <<"tb:" <<tb[index] << endl;
	v01[index] = as[index] * tb0[index]; //线性运动速度

	//Last Segment
	if (pos_end > pos_waypoint)
		ae[index] = -mAcc * 1000;
	else
		ae[index] = mAcc * 1000;

	if (index == 2)
		ae[index] /= 20;

	tb2[index] = td[index] - sqrt(td[index] * td[index] + 2 * (pos_end - pos_waypoint) / ae[index]); //第一段的加速时间，我们预设两端轨迹时间相等
	//cout <<"tb:" <<tb[index] << endl;
	v12[index] = -ae[index] * tb2[index]; //线性运动速度

	//Middle Blend
	if (v12[index] > v01[index])
		aw[index] = mAcc * 1000;
	else
		aw[index] = -mAcc * 1000;

	if (index == 2)
		aw[index] /= 20;

	tb1[index] = (v12[index] - v01[index]) / aw[index];

	//把第一段和第二段的线性时间算出来
	t01[index] = td[index] - tb0[index] - 0.5 * tb1[index];
	t12[index] = td[index] - tb2[index] - 0.5 * tb1[index];

	//cout << "LFPB" << endl;
	//cout << index << ": " << pos_waypoint << endl;
}
void CHLMotionPlan::GetPlanPoints_line(int part_num)
{
	string cart,dicar;
	cart = (string)"./cart_zyz" + to_string(part_num) + (string)".txt";
	dicar = (string)"./dicar" + to_string(part_num) + (string)".txt";

	//创建文件部分
	ofstream outfile;               			//创建文件
	//outfile.open("./cart_zyz.txt");
	outfile.open(cart.c_str()); //例如打开./cart_zyz0.txt

	ofstream outfile_dicar;
	//outfile_dicar.open("./dicar.txt");
	outfile_dicar.open(dicar.c_str());


	

	//通过规划得到每一段的曲线参数
	double start_x = mStartMatrixData[3]*1000;
	double start_y = mStartMatrixData[7]*1000;
	double start_z = mStartMatrixData[11]*1000;
	double end_x = mEndMatrixData[3] * 1000;
	double end_y = mEndMatrixData[7] * 1000;
	double end_z = mEndMatrixData[11] * 1000;
	LFPB_Planning(start_x, end_x, 0);
	LFPB_Planning(start_y, end_y, 1);
	LFPB_Planning(start_z, end_z, 2);
	LFPB_Planning(start_yaw, end_yaw,3);
	LFPB_Planning(start_pitch, end_pitch, 4);
	LFPB_Planning(start_roll, end_roll, 5);

	
	//Only Go Up Segment
	for (int i = 0; i < td[0] / mSampleTime; i++)
	{
		PosStruct temp;
		double t = mSampleTime * i;
		temp.x = start_x;
		temp.y = start_y;

		if (t < tb0[2])
			temp.z = 0.5 * as[2] * t * t + start_z;
		else if (t >= tb0[2] && t < (tb0[2] + t01[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + (t - tb0[2]) * v01[2] + start_z;
		else if (t >= (tb0[2] + t01[2]) && t < (tb0[2] + t01[2] + tb1[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + v01[2] * (t - tb0[2] - t01[2]) + 0.5 * aw[2] * (t - tb0[2] - t01[2]) * (t - tb0[2] - t01[2]) + start_z;
		else if (t >= (tb0[2] + t01[2] + tb1[2]) && t < (tb0[2] + t01[2] + tb1[2] + t12[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + (v01[2] * tb1[2] + 0.5 * aw[2] * pow(tb1[2], 2)) + v12[2] * (t - tb0[2] - t01[2] - tb1[2]) + start_z;
		else
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + (v01[2] * tb1[2] + 0.5 * aw[2] * pow(tb1[2], 2)) + v12[2] * t12[2] + v12[2] * (t - tb0[2] - t01[2] - tb1[2] - t12[2]) + 0.5 * ae[2] * pow((t - tb0[2] - t01[2] - tb1[2] - t12[2]), 2) + start_z;

		temp.yaw = start_yaw;
		temp.pitch = start_pitch;
		temp.roll = start_roll;

		double angle1, angle2, angle3, angle4, angle5, angle6;
		HLRobot::SetRobotEndPos(temp.x, temp.y, temp.z, temp.yaw, temp.pitch, temp.roll);
		HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);


		//cout << temp.x <<" "<<temp.y<<" "<<temp.z<<" "<< temp.yaw<<" "<<temp.pitch<<" "<< temp.roll<< endl;
		outfile << angle1 << "  "
			<< angle2 << "  "
			<< angle3 << "  "
			<< angle4 << "  "
			<< angle5 << "  "
			<< angle6 << "  " << endl;

		outfile_dicar << temp.x << "  "
			<< temp.y << "  "
			<< temp.z << "  "
			<< temp.yaw << "  "
			<< temp.pitch << "  "
			<< temp.roll << "  " << endl;
	}
	//Mid Segment
	for (int i = 0; i < 2 * td[0] / mSampleTime; i++)
	{
		//cout <<" here !" << endl;
		PosStruct temp;
		double t = mSampleTime * i;

		if (t < tb0[0])
			temp.x = 0.5 * as[0] * t * t + start_x;
		else if (t >= tb0[0] && t < (tb0[0] + t01[0]))
			temp.x = 0.5 * as[0] * pow(tb0[0], 2) + (t - tb0[0]) * v01[0] + start_x;
		else if (t >= (tb0[0] + t01[0]) && t < (tb0[0] + t01[0]+ tb1[0]))
			temp.x = 0.5 * as[0] * pow(tb0[0], 2) + t01[0] * v01[0] + v01[0] * (t - tb0[0] - t01[0]) + 0.5 * aw[0] * (t - tb0[0] - t01[0]) * (t - tb0[0] - t01[0]) + start_x;
		else if (t >= (tb0[0] + t01[0] + tb1[0]) && t < (tb0[0] + t01[0] + tb1[0] + t12[0]))
			temp.x = 0.5 * as[0] * pow(tb0[0], 2) + t01[0] * v01[0] + (v01[0] * tb1[0] + 0.5 * aw[0] * pow(tb1[0], 2)) + v12[0] * (t- tb0[0] - t01[0] - tb1[0]) + start_x;
		else
			temp.x = 0.5 * as[0] * pow(tb0[0], 2) + t01[0] * v01[0] + (v01[0] * tb1[0] + 0.5 * aw[0] * pow(tb1[0], 2)) + v12[0] * t12[0] + v12[0] * (t - tb0[0] - t01[0] - tb1[0] - t12[0]) + 0.5*ae[0]*pow((t - tb0[0] - t01[0] - tb1[0] - t12[0]),2) + start_x;

		
		if (t < tb0[1])
			temp.y = 0.5 * as[1] * t * t + start_y;
		else if (t >= tb0[1] && t < (tb0[1] + t01[1]))
			temp.y = 0.5 * as[1] * pow(tb0[1], 2) + (t - tb0[1]) * v01[1] + start_y;
		else if (t >= (tb0[1] + t01[1]) && t < (tb0[1] + t01[1] + tb1[1]))
			temp.y = 0.5 * as[1] * pow(tb0[1], 2) + t01[1] * v01[1] + v01[1] * (t - tb0[1] - t01[1]) + 0.5 * aw[1] * (t - tb0[1] - t01[1]) * (t - tb0[1] - t01[1]) + start_y;
		else if (t >= (tb0[1] + t01[1] + tb1[1]) && t < (tb0[1] + t01[1] + tb1[1] + t12[1]))
			temp.y = 0.5 * as[1] * pow(tb0[1], 2) + t01[1] * v01[1] + (v01[1] * tb1[1] + 0.5 * aw[1] * pow(tb1[1], 2)) + v12[1] * (t - tb0[1] - t01[1] - tb1[1]) + start_y;
		else
			temp.y = 0.5 * as[1] * pow(tb0[1], 2) + t01[1] * v01[1] + (v01[1] * tb1[1] + 0.5 * aw[1] * pow(tb1[1], 2)) + v12[1] * t12[1] + v12[1] * (t - tb0[1] - t01[1] - tb1[1] - t12[1]) + 0.5 * ae[1] * pow((t - tb0[1] - t01[1] - tb1[1] - t12[1]), 2) + start_y;

		double tz = t + td[0]; //为了能够保持先动的z和后动的xy时间一致
		if (tz < tb0[2])
			temp.z = 0.5 * as[2] * tz * tz + start_z;
		else if (tz >= tb0[2] && tz < (tb0[2] + t01[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + (tz - tb0[2]) * v01[2] + start_z;
		else if (tz >= (tb0[2] + t01[2]) && tz < (tb0[2] + t01[2] + tb1[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + v01[2] * (tz - tb0[2] - t01[2]) + 0.5 * aw[2] * (tz - tb0[2] - t01[2]) * (tz - tb0[2] - t01[2]) + start_z;
		else if (tz >= (tb0[2] + t01[2] + tb1[2]) && tz < (tb0[2] + t01[2] + tb1[2] + t12[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + (v01[2] * tb1[2] + 0.5 * aw[2] * pow(tb1[2], 2)) + v12[2] * (tz - tb0[2] - t01[2] - tb1[2]) + start_z;
		else
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + (v01[2] * tb1[2] + 0.5 * aw[2] * pow(tb1[2], 2)) + v12[2] * t12[2] + v12[2] * (tz - tb0[2] - t01[2] - tb1[2] - t12[2]) + 0.5 * ae[2] * pow((tz - tb0[2] - t01[2] - tb1[2] - t12[2]), 2) + start_z;

		if (t < tb0[3])
			temp.yaw = 0.5 * as[3] * t * t + start_yaw;
		else if (t >= tb0[3] && t < (tb0[3] + t01[3]))
			temp.yaw = 0.5 * as[3] * pow(tb0[3], 2) + (t - tb0[3]) * v01[3] + start_yaw;
		else if (t >= (tb0[3] + t01[3]) && t < (tb0[3] + t01[3] + tb1[3]))
			temp.yaw = 0.5 * as[3] * pow(tb0[3], 2) + t01[3] * v01[3] + v01[3] * (t - tb0[3] - t01[3]) + 0.5 * aw[3] * (t - tb0[3] - t01[3]) * (t - tb0[3] - t01[3]) + start_yaw;
		else if (t >= (tb0[3] + t01[3] + tb1[3]) && t < (tb0[3] + t01[3] + tb1[3] + t12[3]))
			temp.yaw = 0.5 * as[3] * pow(tb0[3], 2) + t01[3] * v01[3] + (v01[3] * tb1[3] + 0.5 * aw[3] * pow(tb1[3], 2)) + v12[3] * (t - tb0[3] - t01[3] - tb1[3]) + start_yaw;
		else
			temp.yaw = 0.5 * as[3] * pow(tb0[3], 2) + t01[3] * v01[3] + (v01[3] * tb1[3] + 0.5 * aw[3] * pow(tb1[3], 2)) + v12[3] * t12[3] + v12[3] * (t - tb0[3] - t01[3] - tb1[3] - t12[3]) + 0.5 * ae[3] * pow((t - tb0[3] - t01[3] - tb1[3] - t12[3]), 2) + start_yaw;

		if (t < tb0[4])
			temp.pitch = 0.5 * as[4] * t * t + start_pitch;
		else if (t >= tb0[4] && t < (tb0[4] + t01[4]))
			temp.pitch = 0.5 * as[4] * pow(tb0[4], 2) + (t - tb0[4]) * v01[4] + start_pitch;
		else if (t >= (tb0[4] + t01[4]) && t < (tb0[4] + t01[4] + tb1[4]))
			temp.pitch = 0.5 * as[4] * pow(tb0[4], 2) + t01[4] * v01[4] + v01[4] * (t - tb0[4] - t01[4]) + 0.5 * aw[4] * (t - tb0[4] - t01[4]) * (t - tb0[4] - t01[4]) + start_pitch;
		else if (t >= (tb0[4] + t01[4] + tb1[4]) && t < (tb0[4] + t01[4] + tb1[4] + t12[4]))
			temp.pitch = 0.5 * as[4] * pow(tb0[4], 2) + t01[4] * v01[4] + (v01[4] * tb1[4] + 0.5 * aw[4] * pow(tb1[4], 2)) + v12[4] * (t - tb0[4] - t01[4] - tb1[4]) + start_pitch;
		else
			temp.pitch = 0.5 * as[4] * pow(tb0[4], 2) + t01[4] * v01[4] + (v01[4] * tb1[4] + 0.5 * aw[4] * pow(tb1[4], 2)) + v12[4] * t12[4] + v12[4] * (t - tb0[4] - t01[4] - tb1[4] - t12[4]) + 0.5 * ae[4] * pow((t - tb0[4] - t01[4] - tb1[4] - t12[4]), 2) + start_pitch;

		if (t < tb0[5])
			temp.roll = 0.5 * as[5] * t * t + start_roll;
		else if (t >= tb0[5] && t < (tb0[5] + t01[5]))
			temp.roll = 0.5 * as[5] * pow(tb0[5], 2) + (t - tb0[5]) * v01[5] + start_roll;
		else if (t >= (tb0[5] + t01[5]) && t < (tb0[5] + t01[5] + tb1[5]))
			temp.roll = 0.5 * as[5] * pow(tb0[5], 2) + t01[5] * v01[5] + v01[5] * (t - tb0[5] - t01[5]) + 0.5 * aw[5] * (t - tb0[5] - t01[5]) * (t - tb0[5] - t01[5]) + start_roll;
		else if (t >= (tb0[5] + t01[5] + tb1[5]) && t < (tb0[5] + t01[5] + tb1[5] + t12[5]))
			temp.roll = 0.5 * as[5] * pow(tb0[5], 2) + t01[5] * v01[5] + (v01[5] * tb1[5] + 0.5 * aw[5] * pow(tb1[5], 2)) + v12[5] * (t - tb0[5] - t01[5] - tb1[5]) + start_roll;
		else
			temp.roll = 0.5 * as[5] * pow(tb0[5], 2) + t01[5] * v01[5] + (v01[5] * tb1[5] + 0.5 * aw[5] * pow(tb1[5], 2)) + v12[5] * t12[5] + v12[5] * (t - tb0[5] - t01[5] - tb1[5] - t12[5]) + 0.5 * ae[5] * pow((t - tb0[5] - t01[5] - tb1[5] - t12[5]), 2) + start_roll;


		double angle1, angle2, angle3, angle4, angle5, angle6;
		HLRobot::SetRobotEndPos(temp.x, temp.y, temp.z, temp.yaw, temp.pitch, temp.roll);
		HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);

		
		//cout << temp.x <<" "<<temp.y<<" "<<temp.z<<" "<< temp.yaw<<" "<<temp.pitch<<" "<< temp.roll<< endl;
		outfile << angle1 << "  "
			<< angle2 << "  "
			<< angle3 << "  "
			<< angle4 << "  "
			<< angle5 << "  "
			<< angle6 << "  " << endl;

		outfile_dicar << temp.x << "  "
			<< temp.y << "  "
			<< temp.z << "  " 
			<< temp.yaw << "  "
			<< temp.pitch << "  "
			<< temp.roll << "  "<<endl;
		//Point.push_back(temp);
	}
	
	//Only Go Down Segment
	for (int i = 0; i < td[0] / mSampleTime; i++)
	{
		PosStruct temp;
		double t = mSampleTime * i;
		temp.x = end_x;
		temp.y = end_y;

		double tz = t + 3 * td[0]; //为了能够保持先动的z和后动的xy时间一致
		if (tz < tb0[2])
			temp.z = 0.5 * as[2] * tz * tz + start_z;
		else if (tz >= tb0[2] && tz < (tb0[2] + t01[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + (tz - tb0[2]) * v01[2] + start_z;
		else if (tz >= (tb0[2] + t01[2]) && tz < (tb0[2] + t01[2] + tb1[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + v01[2] * (tz - tb0[2] - t01[2]) + 0.5 * aw[2] * (tz - tb0[2] - t01[2]) * (tz - tb0[2] - t01[2]) + start_z;
		else if (tz >= (tb0[2] + t01[2] + tb1[2]) && tz < (tb0[2] + t01[2] + tb1[2] + t12[2]))
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + (v01[2] * tb1[2] + 0.5 * aw[2] * pow(tb1[2], 2)) + v12[2] * (tz - tb0[2] - t01[2] - tb1[2]) + start_z;
		else
			temp.z = 0.5 * as[2] * pow(tb0[2], 2) + t01[2] * v01[2] + (v01[2] * tb1[2] + 0.5 * aw[2] * pow(tb1[2], 2)) + v12[2] * t12[2] + v12[2] * (tz - tb0[2] - t01[2] - tb1[2] - t12[2]) + 0.5 * ae[2] * pow((tz - tb0[2] - t01[2] - tb1[2] - t12[2]), 2) + start_z;

		temp.yaw = end_yaw;
		temp.pitch = end_pitch;
		temp.roll = end_roll;

		double angle1, angle2, angle3, angle4, angle5, angle6;
		HLRobot::SetRobotEndPos(temp.x, temp.y, temp.z, temp.yaw, temp.pitch, temp.roll);
		HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);


		//cout << temp.x <<" "<<temp.y<<" "<<temp.z<<" "<< temp.yaw<<" "<<temp.pitch<<" "<< temp.roll<< endl;
		outfile << angle1 << "  "
			<< angle2 << "  "
			<< angle3 << "  "
			<< angle4 << "  "
			<< angle5 << "  "
			<< angle6 << "  " << endl;

		outfile_dicar << temp.x << "  "
			<< temp.y << "  "
			<< temp.z << "  "
			<< temp.yaw << "  "
			<< temp.pitch << "  "
			<< temp.roll << "  " << endl;
	}

	outfile << endl;//保存所有数据
}