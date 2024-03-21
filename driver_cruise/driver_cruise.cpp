/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Description: User Module for CyberCruise							█
█	作者: 杨辰兮 & ChatGPT												█
█	联系方式: yangchenxi@sjtu.edu.cn										█
█	日期: 2024.02.27							    						█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	贴士:	您可以折叠 #pragma region 和	#pragma endregion 之间的代码		█
█	这可以使您获得一次性折叠完成的程序块而不是一个函数的能力					█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	DLL接口部分，您可以跳过这部分不阅读									█
█	不要修改这个 #pragma region 中的任何代码!								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< 不要修改这个 region 中的任何代码!
#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <ostream>
#include <fstream>

#include "class_Visualization.h"
#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

//Global variables for vehicle states
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm)
{
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}
#pragma endregion >>>

/*
	上下确界约束函数									 					
	您需要理解它的功能，建议不要修改										
*/
#pragma region <<< Boundaries of control	
double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}
#pragma endregion

/*
	赛道曲率半径计算函数													
	您需要理解它的功能，建议不要修改										
*/
#pragma region <<< Radius of curvature
//		Given three points ahead, outputs a struct circle.				
//		{radius:[1,1000], sign{-1:left,1:right}							
typedef struct Circle
{
	double r;
	int sign;
}circle;

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b * f - e * c) / (b * d - e * a);
	y = (d * c - a * f) / (b * d - e * a);
	r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 1000.0, r);
	int sign = (x > 0) ? 1 : -1;
	return { r,sign };
}
#pragma endregion >>>

/*
	以下是核心控制程序													
	主要输入: _midline, _speed											
	次要输入: _yaw, _yawrate, _acc, _width, _rpm,	_gearbox			
	主要输出: *cmdAcc, *cmdBrake, *cmdSteer  							
	次要输出: *cmdGear 【本样例中已实现】									
	详细信息请参见用户手册												
*/

//基于OpenCV的可视化工具，详情请见文档
cls_VISUAL cls_visual;

/*
	手动换挡程序															
	可以不用看懂，建议不要修改，除非您是学(Juan)霸(Wang) :P					
*/
#pragma region <<< Manual Gear
const float fGearShift[2][4] = //0 for downshift, 1 for upshift
{
	0,105,142,190,
	0,120,160,204
};
void updateGear(int* cmdGear)
{

	//if (_speed > fGearShift[1][_gearbox] && _gearbox < 7) //upshift
	if (_rpm > 650 && _gearbox < 7) //upshift
	{
		*cmdGear = _gearbox + 1;
	}
	//else if (_speed < fGearShift[0][_gearbox - 1] && _gearbox > 1) //downshift
	else if (_rpm < 400 && _gearbox > 1) //downshift
	{
		*cmdGear = _gearbox - 1;
	}
	else
	{
		*cmdGear = _gearbox;
	}
}
#pragma endregion >>>

/*
	PID控制器，由ChatGPT生成												
	可选择性修改，需要完全理解												
*/
class PIDController
{
public:
	double kp, ki, kd;		// PID控制器的参数
	double targetValue;		// 目标值
	double lastError;		// 上一次误差值
	double errorIntegral;	// 误差积分值
	double more_pid_p;
	double more_pid_d;
	double last_dist_error;
	double dist;

public:
	void initial(double p, double i, double d, double target, double pid_p,double pid_d)
	{
		kp = p;
		ki = i;
		kd = d;
		more_pid_p = pid_p;
		more_pid_d = pid_d;
		targetValue = target;
		lastError = 0;
		errorIntegral = 0;
		dist = 0;
	}

	double calculate(double input, double dist_to_error)
	{
		double error = targetValue - input;
		double derivative = error - lastError;
		double derivative2 = dist - last_dist_error;
		errorIntegral += error;
		last_dist_error = dist;
		lastError = error;
		dist += dist_to_error;

		return kp * error + ki * errorIntegral + kd * derivative + more_pid_p * dist+more_pid_d* derivative2;
	}
	void clear_error() {
		errorIntegral = 0;
		dist = 0;
	}

	void change_PID(double p, double i, double d,double pid, double pid_d) {
		kp = p;
		ki = i;
		kd = d;
		more_pid_p = pid;
		more_pid_d = pid_d;
	}
};//高速下方向调整减小

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	平滑突变的输出												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
class smoother {
public :
	double last_val;
	double smooth;
	void initial(double smooth_) {
		smooth = smooth_;
		last_val = 40;
	}
	double calculate(double target_aim) {
		if (target_aim > last_val) {
			last_val += smooth;
			return last_val;
		}
		else if (target_aim < last_val) {
			last_val -= smooth;
			return last_val;
		}
		else {
			return target_aim;
		}
	}
};

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	车辆控制主程序，由ChatGPT自动生成助教完善								█
█	样例代码仅供参考，请在下方设计实现您的算法								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
smoother aim_smoother;
PIDController angleController;	//舵角PID控制
double lastTargetSpeed = 999.0;	//上一帧目标速度
bool isFirstFrame = true;		//第一帧标志
double FirstTenFrame = 0;
double last_speed;
bool clear_sign = 0;
bool straight=1;

/*
根据弯角判断前方瞄点的距离，曲率越大，瞄点越远
*/
int judge_corner() {
	circle curCruve = getR(_midline[30][0], _midline[30][1], _midline[31][0], _midline[31][1], _midline[32][0], _midline[32][1]);	
	circle minCruve=curCruve;
	circle maxCruve= curCruve;
	if (curCruve.r == 1000) {
		straight = 1;
		return 50;
	}
	int i=30;
	//考虑未来200米的情况，识别未来200米的弯角
	while(maxCruve.sign* curCruve.sign>0&& maxCruve.r<1000&&i<=197){
		i++;
		auto cruve = getR(_midline[i][0], _midline[i][1], _midline[i+1][0], _midline[i+1][1], _midline[i+2][0], _midline[i+2][1]);
		if (cruve.r < minCruve.r)
			minCruve = cruve;
		if (cruve.r > maxCruve.r || cruve.sign* maxCruve.sign < 0)
			maxCruve = cruve;
	}
	if (minCruve.r > 500)
		return  50;
	else if(minCruve.r>100)
		return  constrain(0,50,40+minCruve.r*0.1-10);
	else
		return  20+minCruve.r*0.2;
}



static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	double max_yawrate = 0;
	double cal_the_error = 0;

	cal_the_error = sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);
	if (_midline[0][0] < 0)
		cal_the_error *= -1;
	//计算error值，在后文输出error值以考察算法性能
	double targetAngleError = 0;

	/*短暂加速后通过末速度确定不同赛道特性
	末速度last_speed大于10为铺装路面
	小于10为土路	*/
	if (FirstTenFrame < 50) {
		*cmdAcc = 1;
		FirstTenFrame++;
		if (FirstTenFrame == 49)
			last_speed = _speed;
	}
	else {
		//铺装路面
		//if (last_speed > 10.7) {
		if (last_speed > 10.7) {
			if (isFirstFrame)
			{
				isFirstFrame = false;
				//angleController.initial(3, 0.001, 4, targetAngleError);
				angleController.initial(4, 0,20, targetAngleError, 0,0);
				aim_smoother.initial(0.25);
			}
#pragma region <<< cal_targetSpeed
			double targetSpeed;  //目标车速
			double currentSpeed = _speed;	//当前误差
			circle myCurve;
			float minCruve = 1000.0;
			circle curCruve = getR(_midline[0][0], _midline[0][1], _midline[1][0], _midline[1][1], _midline[2][0], _midline[2][1]);

			for (int fStep = 0; fStep < constrain(0, 180, abs(_speed * _speed * 0.0042)); fStep++)
				//根据速度决定检测距离
			{
				myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
				if (myCurve.r < minCruve)
				{
					minCruve = myCurve.r;
				}
			}
			if (minCruve > 900) {
				//angleController.change_k(4, 0.0006, 4);
				//if (minCruve > 480)
				targetSpeed = 300;
			}
			else if (minCruve > 200) {
				targetSpeed = 250 + minCruve * 1 - 1 * 200;
				//angleController.change_k(4, 0.0006, 4);
			}
			else if (minCruve > 100) {
				targetSpeed = 140 + minCruve *0.5  - 0.5 * 100;
				//angleController.change_k(4, 0.0006, 4);
			}
			else {
				targetSpeed = 60 + minCruve*0.8 ;
				//angleController.change_k(4, 0.0006, 4);
			}

#pragma endregion >>>
#pragma region <<< cal_steer
			int target_aim = 30;
			target_aim = judge_corner();
			double aim;
			aim = aim_smoother.calculate(target_aim);
			int ans = abs(aim);//确定瞄点
			double currentAngleError;//当前误差
			currentAngleError = atan2(_midline[ans][0], _midline[ans][1]);
			*cmdSteer = constrain(-1, 1, angleController.calculate(currentAngleError, cal_the_error));			
#pragma endregion >>>
			//根据当前速度和目标速度关系，控制油门刹车以改变速度
			//当前速度低于目标，需要加速
			if (currentSpeed < targetSpeed)  
			{
#pragma region <<< accelarate with tcs
				//在低速情况下，避免过大油门造成车辆的打滑，简单使用速度的线性关系计算油门开度
				if (abs(*cmdSteer) < 0.015 && targetSpeed > 100 ) {
					*cmdAcc = constrain(0, 1, 0.3 + _speed * 0.008);
				}
				else
				{
					*cmdAcc = 0.3;
				}
				//加速情况下，刹车为0
				*cmdBrake = 0;
#pragma endregion >>>
			}
			//需要刹车
			else
			{
#pragma region <<< brake with abs
				/*减速情况下，刹车
				增加ABS防抱死刹车功能，不同速度下刹车力度不同以防止车轮抱死，
				进而提高刹车性能，同时保持车辆转向能力*/
				*cmdAcc = 0;
				if (_speed > 170)
					*cmdBrake = constrain(0, 1, 0.725 + _speed * 0.005 - 170 * 0.005);
				else if (_speed > 160)
					*cmdBrake = 0.7;
				else if (_speed > 150)
					*cmdBrake = 0.675;
				else if (_speed > 140)
					*cmdBrake = 0.60 + _speed * 0.005 - 140 * 0.005;
				else if (_speed > 130)
					*cmdBrake = 0.58 + _speed * 0.002 - 130 * 0.002;
				else if (_speed > 120)
					*cmdBrake = 0.55 + _speed * 0.003 - 120 * 0.003;
				else if (_speed > 110)
					*cmdBrake = 0.55;
				else if (_speed > 100)
					*cmdBrake = 0.5;
				else if (_speed > 90)
					*cmdBrake = 0.45;
				else if (_speed > 80)
					*cmdBrake = 0.42;
				else if (_speed > 70)
					*cmdBrake = 0.38;
				else
					*cmdBrake = 0.3;

#pragma endregion >>>
			}
			cls_visual.Fig2Y(1, 0, 300, 0, 1000, 10, "aim", aim, "minCruve", minCruve, "targetSpeed ", targetSpeed);
		}
		//土路
		else {
			if (isFirstFrame)
			{
				isFirstFrame = false;
				angleController.initial(4, 0, 20, targetAngleError, 0,0);
				aim_smoother.initial(0.25);
			}
#pragma region <<< cal_targetSpeed
			double targetSpeed;  //目标车速
			double currentSpeed = _speed;	//当前误差
			circle myCurve;
			float minCruve = 1000.0;
			float minCruve_100 = 500;
			for (int fStep = 0; fStep < constrain(0, 180, abs(_speed * _speed * 0.006)); fStep++)
				//尤里卡，根据速度决定判断距离
			{
				myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
				if (myCurve.r < minCruve)
				{
					minCruve = myCurve.r;
				}
			}
			if (minCruve > 900)
				targetSpeed = 300;
			else if (minCruve > 200)
				targetSpeed = 155+ minCruve * 0.3 - 100 * 0.3; 
			else if (minCruve > 100)
				targetSpeed = 125+ minCruve*0.3-100*0.3;
			else
				targetSpeed = 50 + minCruve * 0.75;
#pragma endregion >>>
#pragma region <<< cal_steer
			double currentAngleError;//当前误差
			int target_aim = 30;
			target_aim = judge_corner();
			double aim;
			aim = aim_smoother.calculate(target_aim);
			int ans = abs(aim);
			currentAngleError = atan2(_midline[ans][0], _midline[ans][1]);
			*cmdSteer = constrain(-1, 1, angleController.calculate(currentAngleError, cal_the_error));
			//舵角PID控制
#pragma endregion >>>
			//根据当前速度和目标速度关系，控制油门刹车以改变速度
			if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
			{
#pragma region <<< accelarate with tcs
				if (abs(*cmdSteer) < 0.015 && targetSpeed > 100 ) {
					*cmdAcc = constrain(0, 1, 0.3 + _speed * 0.007);
				}//在低速情况下，避免过大油门造成车辆的打滑，简单使用速度的线性关系计算油门开度
				else
				{
					//*cmdAcc = constrain(0, 1, 0.2 + _speed * 0.003);
					*cmdAcc = 0.3;
				}
				//加速情况下，刹车为0
				*cmdBrake = 0;
#pragma endregion >>>
			}
			else
			{
#pragma region <<< brake with abs
				//* cmdAcc = 0;
				//*cmdBrake = 1;
				/*减速情况下，刹车
				增加ABS防抱死刹车功能，不同速度下刹车力度不同以防止车轮抱死，
				进而提高刹车性能，同时保持车辆转向能力*/
				* cmdAcc = 0;
				if (_speed > 170)
					*cmdBrake = constrain(0, 1, 0.725 + _speed * 0.005 - 170 * 0.005);
				else if (_speed > 160)
					*cmdBrake = 0.7;
				else if (_speed > 150)
					*cmdBrake = 0.675;
				else if (_speed > 140)
					*cmdBrake = 0.60 + _speed * 0.005 - 140 * 0.005;
				else if (_speed > 130)
					*cmdBrake = 0.58 + _speed * 0.002 - 130 * 0.002;
				else if (_speed > 120)
					*cmdBrake = 0.55 + _speed * 0.003 - 120 * 0.003;
				else if (_speed > 110)
					*cmdBrake = 0.55;
				else if (_speed > 100)
					*cmdBrake = 0.5;
				else if (_speed > 90)
					*cmdBrake = 0.45;
				else if (_speed > 80)
					*cmdBrake = 0.42;
				else if (_speed > 70)
					*cmdBrake = 0.38;
				else
					*cmdBrake = 0.3;

#pragma endregion >>>
			}
			cls_visual.Fig2Y(1, 0, 300, 0, 1000, 10, "aim", aim, "minCruve", minCruve, "targetSpeed ", targetSpeed);
		}

	}
	//更新档位
	updateGear(cmdGear);
	//窗口可视化minCruve

}
