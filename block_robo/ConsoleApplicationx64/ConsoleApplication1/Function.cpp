
#include<winsock.h>
#include <conio.h>
#pragma comment(lib,"ws2_32.lib")
using namespace std;
#pragma comment(lib, "WS2_32.lib")
#include "Function.h"
#include "RecDect.h"
#include<iostream>
#include"MotionPlan.h"
#include"FtpControl.h"





//定义长度变量
int send_len = 0;
int recv_len = 0;
//定义发送缓冲区和接受缓冲区
char send_buf[100] = {};
char recv_buf[200] = {};
string recvstr;
//定义服务端套接字，接受请求套接字
SOCKET s_server;
//服务端地址客户端地址
SOCKADDR_IN server_addr;

void RobotConnect()
{
	
	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}
}

void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}
	//填充服务端地址信息

}



void close()
{
	//关闭套接字
	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
}

void Login2robo()
{
	send_len = send(s_server, "[1# System.Login 0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << "Login" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << "PowerEnable" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[3# System.Abort 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << "Abort" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);
	send_len = send(s_server, "[4# System.Start 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << "Start" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);
	send_len = send(s_server, "[5# Robot.Home 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << "Home" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[6# System.Auto 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << "Auto" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[6# System.Speed 90]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << "Speed" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[8# Robot.Frame 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << "Robot.Frame" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);

	send_len = send(s_server, "[9# IO.Set DOUT(20103),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);

	send_len = send(s_server, "[10# IO.Set DOUT(20104),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	//PPB使能
	send_len = send(s_server, "[7# PPB.Enable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[PPB_Enable]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	
}

bool traj_Generate(PosStruct start_pt, PosStruct end_pt, int part_num)
{
	cout << "开始规划第"<<part_num<<"段曲线" << endl;
	//梯型速度规划
	CHLMotionPlan trajectory1;
	trajectory1.SetPlanPoints(start_pt, end_pt);//单位分别为mm，degree
	trajectory1.SetProfile(10, 1, 10);    //vel °/s， acc °/s.s, dec °/s.s
	trajectory1.SetSampleTime(0.004);      //s
	//trajectory1.GetPlanPoints();           //关节空间梯形速度规划
	trajectory1.GetPlanPoints_line(part_num);      //笛卡尔空间直线轨迹梯形速度规划 

	cout << "结束规划第" << part_num << "段曲线" <<endl;
	return true;
}

bool move2catch(int part_num)
{
	/*send_len = send(s_server, "[14# WaitTime 1000]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "delay 1000" << endl;
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);*/
	//为了保险起见，我们将松爪子放在这里
	send_len = send(s_server, "[1# IO.Set DOUT(20104),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[stop XI]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	send_len = send(s_server, "[2# IO.Set DOUT(20103),1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[expell]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//send_len = send(s_server, "[14# WaitTime 2000]", 100, 0);
	//recv_len = recv(s_server, recv_buf, 200, 0);
	//cout << "delay 1000" << endl;
	//cout << recv_buf << endl;
	//memset(recv_buf, '\0', sizeof(recv_buf));
	//Sleep(500);
	send_len = send(s_server, "[14# IO.Set DOUT(20103),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);

	//send_len = send(s_server, "[14# IO.Set DOUT(20104),0]", 100, 0);
	//recv_len = recv(s_server, recv_buf, 200, 0);
	//cout << recv_buf << endl;
	//memset(recv_buf, '\0', sizeof(recv_buf));
	//Sleep(200);
	//cout << "双0" << endl;



	//以下均为读取轨迹文件并沿着执行的过程
	string cart,sever,readsever;
	cart = (string)"./cart_zyz" + to_string(part_num) + (string)".txt";
	sever = (string)"severdata" + to_string(part_num) + (string)".txt";
	readsever = (string)"[4# PPB.ReadFile 1,/data/severdata" + to_string(part_num) + (string)".txt]";
	FtpControl::Upload("192.168.10.101", "data", cart.c_str(), sever.c_str());

	//PPB读取data文件
	send_len = send(s_server, readsever.c_str(), 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ReadFile]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);

	//到达起始点
	send_len = send(s_server, "[5# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[tostartpoint]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//ppb运行
	send_len = send(s_server, "[6# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[run]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(3500);
	cout << "运行的延迟结束（借此观察一下延迟时间够不够）" << endl;

	/*send_len = send(s_server, "[14# WaitTime 3000]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "delay 1000" << endl;
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);*/


	return true;
}

bool move2place(int part_num)
{

	/*send_len = send(s_server, "[14# WaitTime 1000]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "delay 1000" << endl;
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);*/
	//抓取积木
	send_len = send(s_server, "[1# IO.Set DOUT(20104),1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[XI]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	send_len = send(s_server, "[1# IO.Set DOUT(20103),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[stop spell]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);



	//以下均为读取轨迹文件并沿着执行的过程
	string cart, sever, readsever;
	cart = (string)"./cart_zyz" + to_string(part_num) + (string)".txt";
	sever = (string)"severdata" + to_string(part_num) + (string)".txt";
	readsever = (string)"[4# PPB.ReadFile 1,/data/severdata" + to_string(part_num) + (string)".txt]";
	FtpControl::Upload("192.168.10.101", "data", cart.c_str(), sever.c_str());

	//PPB读取data文件
	send_len = send(s_server, readsever.c_str(), 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ReadFile]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);

	//到达起始点
	send_len = send(s_server, "[3# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[tostartpoint]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//ppb运行
	send_len = send(s_server, "[4# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[run]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(3500);

	cout << "运行的延迟结束（借此观察一下延迟时间够不够）" << endl;


	/*send_len = send(s_server, "[14# WaitTime 2000]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "delay 1000" << endl;
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);*/
	return true;
}

bool end_process()
{
	//松开爪子，回到安全点

	//松开爪子
	send_len = send(s_server, "[0# IO.Set DOUT(20104),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[stop XI]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);
	send_len = send(s_server, "[0# IO.Set DOUT(20103),1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[expell]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);
	send_len = send(s_server, "[0# IO.Set DOUT(20103),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[stop expell]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);


	//回到安全点,暂时我使用自带的home功能，也许不可行，需要换成自己的安全点规划
	send_len = send(s_server, "[5# Robot.Home 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	return true;
}

bool backhome()
{
	send_len = send(s_server, "[0# Robot.Frame 1,2]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[backhome]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	send_len = send(s_server, "[0# Location homept=410,0,800,0,180,138]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[backhome]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	send_len = send(s_server, "[0# Move.Joint homept]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[backhome]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(2000);

	send_len = send(s_server, "[8# Robot.Frame 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << "Robot.Frame" << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);

	return true;
}
