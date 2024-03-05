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

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	上下确界约束函数									 					█
█	您需要理解它的功能，建议不要修改										█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
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

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	赛道曲率半径计算函数													█
█	您需要理解它的功能，建议不要修改										█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
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
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	return { r,sign };
}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	以下是核心控制程序													█
█	主要输入: _midline, _speed											█
█	次要输入: _yaw, _yawrate, _acc, _width, _rpm,	_gearbox			█
█	主要输出: *cmdAcc, *cmdBrake, *cmdSteer  							█
█	次要输出: *cmdGear 【本样例中已实现】									█
█	详细信息请参见用户手册												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

//基于OpenCV的可视化工具，详情请见文档
cls_VISUAL cls_visual;

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	手动换挡程序															█
█	可以不用看懂，建议不要修改，除非您是学(Juan)霸(Wang) :P					█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< Manual Gear
const float fGearShift[2][4] = //0 for downshift, 1 for upshift
{
	0,105,142,190,
	0,120,160,204
};
void updateGear(int* cmdGear)
{

	if (_speed > fGearShift[1][_gearbox] && _gearbox < 7) //upshift
	{
		*cmdGear = _gearbox + 1;
	}
	else if (_speed < fGearShift[0][_gearbox - 1] && _gearbox > 1) //downshift
	{
		*cmdGear = _gearbox - 1;
	}
	else
	{
		*cmdGear = _gearbox;
	}
}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	PID控制器，由ChatGPT生成												█
█	可选择性修改，需要完全理解												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
class PIDController
{
private:
	double kp, ki, kd;		// PID控制器的参数
	double targetValue;		// 目标值
	double lastError;		// 上一次误差值
	double errorIntegral;	// 误差积分值

public:
	void initial(double p, double i, double d, double target)
	{
		kp = p;
		ki = i;
		kd = d;
		targetValue = target;
		lastError = 0;
		errorIntegral = 0;
	}

	double calculate(double input)
	{
		double error = targetValue - input;
		double derivative = error - lastError;
		errorIntegral += error;
		lastError = error;
		return kp * error + ki * errorIntegral + kd * derivative;
	}
};//高速下方向调整减小

	

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	车辆控制主程序，由ChatGPT自动生成助教完善								█
█	样例代码仅供参考，请在下方设计实现您的算法								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
PIDController speedController;	//速度PID控制
PIDController angleController;	//舵角PID控制
double lastTargetSpeed = 999.0;	//上一帧目标速度
bool isFirstFrame = true;		//第一帧标志

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	double cal_the_error = 0;
	cal_the_error = sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);
	//计算error值，在后文输出error值以考察算法性能
#pragma region <<< cal_targetSpeed
	double targetSpeed;  //目标车速
	double currentSpeed = _speed;	//当前误差
	circle myCurve;
	float minCruve = 500.0;
	float minCruve_100 = 500;
	for (int fStep = 0; fStep < 50; fStep++)//先检测前50米
	{
		myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
		if (myCurve.r < minCruve)
		{
			minCruve = myCurve.r;
		}
	}

	minCruve_100 = minCruve;
	for (int fStep = 50; fStep < 100; fStep++)//检测前100米，在速度稍大（100）的情况下，是否有需要急刹的情况
	{
		myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
		//探索未来的赛道的最小曲率
		if (myCurve.r < minCruve_100) {
			minCruve_100 = myCurve.r;
		}
	}
	//设定目标速度，如果前方弯道就设定低，直道就设定高
	if (minCruve > 480)
		targetSpeed = 300;
	else if (minCruve > 200)
		targetSpeed = 90 + minCruve * 0.15 - 0.15 * 200;
	else if (minCruve > 100)
		targetSpeed = 65 + minCruve * 0.25 - 0.25 * 100;
	else
		targetSpeed = 35 + minCruve * 0.25;
	if (minCruve_100 < 200 && minCruve_100 > 120 && targetSpeed > 100 && _speed > 100) {
		targetSpeed = 100;
	}//若探测到前方100米赛道有弯角且自车速度过快，提前刹车
	else if (minCruve_100 <= 120 && targetSpeed > 75 && _speed > 100) {
		targetSpeed = 75;
	}//如上

	for (int fStep = 100; fStep < 180; fStep++)//检测前200米，在速度较大（150）的情况下，是否有需要急刹的情况
	{
		myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
		if (myCurve.r <= 100 && _speed > 150) {
			minCruve = myCurve.r;
			if (targetSpeed > 50)
				targetSpeed = 50;
			break;
		}
		if (myCurve.r <= 200 && _speed > 150) {
			minCruve = myCurve.r;
			if (targetSpeed > 80)
				targetSpeed = 80;
			break;
		}
	}
#pragma endregion >>>
#pragma region <<< cal_steer
	double currentAngleError;//当前误差
	double targetAngleError = 0;
	currentAngleError = atan2((_midline[1][0]+ 2*_midline[0][0])/3, (_midline[1][1] + 2*_midline[0][1]) / 3);
	//尤里卡！！！！！瞄点越近越好，没有近的，那就创造近的！

	//第一帧初始化舵角控制参数，清空积分器和微分器，因为控制目标为恒零，所以只需要初始化一次
	if (isFirstFrame)
	{
		isFirstFrame = false;
		//angleController.initial(3, 0.001, 4, targetAngleError);
		angleController.initial(4, 0.001, 4, targetAngleError);
	}

	//舵角PID控制
	*cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));
#pragma endregion >>>
	//根据当前速度和目标速度关系，控制油门刹车以改变速度
	if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
	{
#pragma region <<< accelarate with tcs
		if (abs(*cmdSteer) < 0.015 && targetSpeed==300&& cal_the_error<0.01) {
			*cmdAcc = constrain(0, 1, 0.3 + _speed * 0.007);
		}//在低速情况下，避免过大油门造成车辆的打滑，简单使用速度的线性关系计算油门开度
		else
		{
			*cmdAcc = 0.3;
		}
		//加速情况下，刹车为0
		*cmdBrake = 0;
#pragma endregion >>>
	}
	else
	{	
#pragma region <<< brake with abs
	/*减速情况下，刹车
	增加ABS防抱死刹车功能，不同速度下刹车力度不同以防止车轮抱死，
	进而提高刹车性能，同时保持车辆转向能力*/
		*cmdAcc = 0;
		if(_speed > 170)
			*cmdBrake=constrain( 0,1, 0.725+_speed*0.005-170 * 0.005);
		else if(_speed>160)
			*cmdBrake = 0.7;
		else if (_speed > 150)
			*cmdBrake = 0.675;
		else if (_speed > 140)
			*cmdBrake = 0.60 + _speed * 0.005 -140 * 0.005;
		else if (_speed > 130)
			*cmdBrake = 0.58+_speed*0.002-130*0.002;
		else if (_speed > 120)
			*cmdBrake = 0.55+_speed * 0.003 - 120 * 0.003;
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
	//更新档位
	updateGear(cmdGear);
	//窗口可视化minCruve
	 cls_visual.Fig2Y(1, 0, 500, 0, 1, 10, "targetSpeed", targetSpeed, "error", cal_the_error,"minCruve" , minCruve);
}
