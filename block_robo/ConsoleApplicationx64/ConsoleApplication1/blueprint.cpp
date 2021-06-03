#include "blueprint.h"
vector<PosStruct> blueprint(int num)
{
	vector<PosStruct> pos_Desired;
	switch (num)
	{
	case 4:
		pos_Desired.push_back(PosStruct{ 410, -150, 460, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 460, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 475, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 475, 0, 180, 138 });
	case 6:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 479, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 479, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 494, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 494, 0, 180, 228 });
		break;
	case 8:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 479, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 479, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 494, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 494, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 509, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 509, 0, 180, 138 });
	case 10:
		pos_Desired.push_back(PosStruct{ 410, -150, 466, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 466, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 479, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 479, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 494, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 494, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 509, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 524, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 539, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 554, 0, 180, 138 });

	case 12:
		pos_Desired.push_back(PosStruct{ 410, -150, 461, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 461, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 476, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 476, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 491, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 491, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 506, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 506, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 521, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 521, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 536, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 536, 0, 180, 138 });
	default:
		break;
	}

	return pos_Desired;
}