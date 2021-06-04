// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。


#include<iostream>

#include "FtpControl.h"
#include "RecDect.h"
#include "Function.h"
#include "MotionPlan.h"
#include "HLrobotconfig.h"
#include "blueprint.h"
using namespace std;
using namespace cv;

vector<PosStruct> block_Vec;//单位暂时是m，degree
vector<double> block_dis;
PosStruct home_pt{ 410,0,800,0,180,138 };

int main()
{   

	RobotConnect();

	Login2robo();
	//回到安全点（同时也是摄像的初始位置）
	//backhome();
	 
	 
	//图像检测部分
	 
	//Step1. 对图像进行检测，获得积木坐标信息
	Get_RGB();
	ColorDect(0, 0);
	
	//Step2. 设置放置点，预计的放置点中心位置为平台的一侧，预设好一堆点位信息，vector容器大小对应于小鱼检测到的积木数的一个定值
	//		 我们先预设为2层积木，即4块
	// 
	int num_detect = block_Vec.size();
	if(num_detect < 4)
	{
		cout << "检测到的积木数量不足" << endl;
		return 0;
	}
	vector<PosStruct> pos_Desired;
	pos_Desired = blueprint(num_detect);
	cout <<"共" <<num_detect <<"个矩形"<< endl;

	//Step3. block_Vec中进行排序，其中包含了每一个积木的中心点位以及rpy信息，准备将其作为起始点进行搭建
	for (int i = 0; i < num_detect; i++)
	{
		for (int j = i + 1; j < num_detect; j++)
		{
			if (    sqrt( pow((block_Vec[i].x - 385), 2) + pow((block_Vec[i].y + 150), 2)) > sqrt(pow((block_Vec[j].x - 385), 2) + pow((block_Vec[j].y + 150), 2)) )
			{
				PosStruct temp;
				temp = block_Vec[i];
				block_Vec[i] = block_Vec[j];
				block_Vec[j] = temp;
			}
		}
	}
	
	//for (int i = 0; i < num_detect; i++)
	//{
	//	cout << block_Vec[i].x << " " << block_Vec[i].y << " "<< sqrt(pow((block_Vec[i].x - 385), 2) + pow((block_Vec[i].y + 150), 2)) << endl;
	//}

	//规划与执行部分

	//我想将S4,S5封装为四个函数，  即函数1：登录上电回零等操作
	//								 函数2：松爪子->移动到检测到的积木点
	//							     函数3：抓取积木->移动到放置点
	//                               函数4：收尾工作：将爪子松开，并移动至一个高处的安全点
	//以上函数将放置在Function.cpp中
	
	
	//规划部分
	traj_Generate(home_pt, block_Vec[0], 0);
	traj_Generate(block_Vec[0], pos_Desired[0], 1);
	for (int i = 0; i < num_detect - 1; i++)
	{
		traj_Generate(pos_Desired[i], block_Vec[i + 1], 2 * i + 2);
		traj_Generate(block_Vec[i + 1], pos_Desired[i + 1], 2 * i + 3);
	}
	
	//执行部分
	for (int i = 0; i < num_detect; i++)
	{
		move2catch(2 * i);
		move2place(2 * i + 1);
	}

	if (end_process())
	{
		cout << "搭建积木完成" << endl;
		return 0;
	}

	////Step4. 进行一一对应，使用规划算法
	////梯型速度规划
	//CHLMotionPlan trajectory1;
	//trajectory1.SetPlanPoints(block_Vec[0], pos_Desired[0]);//单位分别为mm，degree
	//trajectory1.SetProfile(10, 1, 10);    //vel °/s， acc °/s.s, dec °/s.s
	//trajectory1.SetSampleTime(0.001);      //s
	////trajectory1.GetPlanPoints();           //关节空间梯形速度规划
	//trajectory1.GetPlanPoints_line();      //笛卡尔空间直线轨迹梯形速度规划 
	////Step5. 一一执行规划的轨迹
	


	/*ColorDect(0, 0);*/
	/*cout << block_Vec.size() << endl;
	for (int i = 0; i < block_Vec.size(); i++)
	{
		cout << block_Vec[i].x << endl;
	}*/
	

	waitKey();
	return 0;
}

