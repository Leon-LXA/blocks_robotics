#include "blueprint.h"
vector<PosStruct> blueprint(int num)
{
	vector<PosStruct> pos_Desired;
	switch (num)
	{
	case 4:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 479, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 479, 0, 180, 138 });
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
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
	case 9:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 537, 0, 180, 138 });
	case 10:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 537, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 552, 0, 180, 138 });
	case 11:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 537, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 552, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 567, 0, 180, 138 });
	case 12:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 537, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 552, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 567, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 582, 0, 180, 138 });
	case 13:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 537, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 552, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 567, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 582, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 597, 0, 180, 138 });
	case 14:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 537, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 552, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 567, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 582, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 597, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 612, 0, 180, 138 });
	case 15:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 537, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 552, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 567, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 582, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 597, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 612, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 627, 0, 180, 138 });
	case 16:
		pos_Desired.push_back(PosStruct{ 410, -150, 464, 0, 180, 228 });//x,y,z,yaw,pitch,roll
		pos_Desired.push_back(PosStruct{ 360, -150, 464, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -175, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -125, 477, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 410, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 360, -150, 492, 0, 180, 228 });
		pos_Desired.push_back(PosStruct{ 385, -150, 507, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 522, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 537, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 552, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 567, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 582, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 597, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 612, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 627, 0, 180, 138 });
		pos_Desired.push_back(PosStruct{ 385, -150, 642, 0, 180, 138 });
	default:
		break;
	}

	return pos_Desired;
}