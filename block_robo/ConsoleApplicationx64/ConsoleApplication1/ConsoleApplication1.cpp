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
	//Step2. block_Vec中进行排序，其中包含了每一个积木的中心点位以及rpy信息，准备将其作为起始点进行搭建
	// 
	//Step3. 设置放置点，预计的放置点中心位置为平台的一侧，预设好一堆点位信息，vector容器大小对应于小鱼检测到的积木数的一个定值
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
	


	//规划与执行部分

	//我想将S4,S5封装为四个函数，  即函数1：登录上电回零等操作
	//								 函数2：松爪子->移动到检测到的积木点
	//							     函数3：抓取积木->移动到放置点
	//                               函数4：收尾工作：将爪子松开，并移动至一个高处的安全点
	//以上函数将放置在Function.cpp中
	
	
	//规划部分
	traj_Generate(home_pt,        block_Vec[0], 0);
	
	traj_Generate(block_Vec[0], pos_Desired[0], 1);
	
	traj_Generate(pos_Desired[0], block_Vec[1], 2);
	
	traj_Generate(block_Vec[1], pos_Desired[1], 3);
	
	traj_Generate(pos_Desired[1], block_Vec[2], 4);
	
	traj_Generate(block_Vec[2], pos_Desired[2], 5);
	
	traj_Generate(pos_Desired[2], block_Vec[3], 6);
	
	traj_Generate(block_Vec[3], pos_Desired[3], 7);

	traj_Generate(pos_Desired[3], block_Vec[4], 8);

	traj_Generate(block_Vec[4], pos_Desired[4], 9);

	traj_Generate(pos_Desired[4], block_Vec[5], 10);

	traj_Generate(block_Vec[5], pos_Desired[5], 11);

	traj_Generate(pos_Desired[5], block_Vec[6], 12);

	traj_Generate(block_Vec[6], pos_Desired[6], 13);

	traj_Generate(pos_Desired[6], block_Vec[7], 14);

	traj_Generate(block_Vec[7], pos_Desired[7], 15);
	
	traj_Generate(pos_Desired[7], block_Vec[8], 16);

	traj_Generate(block_Vec[8], pos_Desired[8], 17);

	traj_Generate(pos_Desired[8], block_Vec[9], 18);

	traj_Generate(block_Vec[9], pos_Desired[9], 19);

	//执行部分
	move2catch(0);
	move2place(1);
	move2catch(2);
	move2place(3);
	move2catch(4);
	move2place(5);
	move2catch(6);
	move2place(7);
	move2catch(8);
	move2place(9);
	move2catch(10);
	move2place(11);
	move2catch(12);
	move2place(13);
	move2catch(14);
	move2place(15);
	move2catch(16);
	move2place(17);
	move2catch(18);
	move2place(19);

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

