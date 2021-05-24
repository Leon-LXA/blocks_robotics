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
		break;
	default:
		break;
	}

	return pos_Desired;
}