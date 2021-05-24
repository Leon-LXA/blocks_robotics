#pragma once
#include <vector>
#include "RecDect.h"
#include "MotionPlan.h"


//可在此设计相关功能函数，如机器人的、连接初始化、机器人的运动、积木的抓取顺序等

//以机器人连接为例
void RobotConnect();

void initialization();

void close();

//以下是我添加的封装功能函数

//登录等操作
void Login2robo();

//松爪子并移动到抓取点
bool move2catch();

bool move2place();

bool end_process();

bool traj_Generate(PosStruct start_pt, PosStruct end_pt);

bool backhome();