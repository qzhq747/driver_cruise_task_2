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
	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█	舵角控制																█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	double targetAngleError = 0.0; //目标误差
	double currentAngleError = atan2(_midline[5][0], _midline[5][1]); //当前误差
	if (_speed > 150)//不同速度下误差∠需要不同设置
		currentAngleError = atan2(_midline[20][0], _midline[20][1]);
	else if(_speed > 100)
		currentAngleError = atan2(_midline[10][0], _midline[10][1]);

	//第一帧初始化舵角控制参数，清空积分器和微分器，因为控制目标为恒零，所以只需要初始化一次
	if (isFirstFrame)
	{
		isFirstFrame = false;
		angleController.initial(1.0, 0.0, 0.0, targetAngleError);
	}
	
	//舵角PID控制
	*cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));


	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█	速度控制																█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	double targetSpeed;  //目标车速
	double currentSpeed = _speed;	//当前误差
	
	//计算前方是直道还是弯道
	circle myCurve;
	float minCruve = 500.0;
	/*fist version
	for (int fStep = 0; fStep < 50; fStep++)//可以调到180
	{
		myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
		if (myCurve.r < minCruve)
		{
			minCruve = myCurve.r;
		}
	//探索未来的赛道的最小曲率
	}
	//设定目标速度，如果前方弯道就设定低，直道就设定高
	if (minCruve > 300)
		targetSpeed = 250;
	else if (minCruve > 150)
		targetSpeed = 150;
	else if (minCruve > 100)
		targetSpeed = 80;
	else
		targetSpeed = 50;
	*/
	for (int fStep = 0; fStep < 50; fStep++)//先检测前50米
	{
		myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
		if (myCurve.r < minCruve)
		{
			minCruve = myCurve.r;
		}
	}
	for (int fStep = 50; fStep < 100; fStep++)//检测前100米，在速度稍大（100）的情况下，是否有需要急刹的情况
	{
		myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
	//探索未来的赛道的最小曲率
		if (myCurve.r<= 100 && _speed > 100) {
			minCruve = myCurve.r;
			targetSpeed = 50;
			break;
		}
	}
	for (int fStep = 100; fStep < 180; fStep++)//检测前200米，在速度较大（150）的情况下，是否有需要急刹的情况
	{
		myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
		//探索未来的赛道的最小曲率
		if (myCurve.r <= 100 && _speed > 150) {
			minCruve = myCurve.r;
			targetSpeed = 50;
			break;
		}
	}

	//设定目标速度，如果前方弯道就设定低，直道就设定高
	if (minCruve > 300)
		targetSpeed = 180;
	else if (minCruve > 150)
		targetSpeed = 150;
	else if (minCruve > 100)
		targetSpeed = 80;
	else
		targetSpeed = 50;

	//每当目标速度变化时初始化PID控制器，重设参数，清空积分器和微分器
	if (targetSpeed != lastTargetSpeed)
	{
		speedController.initial(0.01, 0.0, 0.0, targetSpeed);
		lastTargetSpeed = targetSpeed;
	}

	//根据当前速度和目标速度关系，控制油门刹车以改变速度
	if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
	{
		
		if (_speed > 100 && abs(*cmdSteer) < 0.1)
		{
			//速度较快且舵角较小时，使用PID控制
			*cmdAcc = constrain(0.0, 1.0, speedController.calculate(currentSpeed));
		}
		else if (_speed > 50 && abs(*cmdSteer) < 0.1)
		{
			//速度较慢且舵角较小时，限定油门
			*cmdAcc = 0.6;
		}
		else
		{
			//除此之外的情况进一步限定油门
			*cmdAcc = 0.3;
		}
		//加速情况下，刹车为0
		*cmdBrake = 0;
	}
	else
	{
		//减速情况下，刹车
		//增加循迹刹车功能
		*cmdAcc = 0;
		double abssteer= *cmdSteer;//转向绝对值
		if (*cmdSteer < 0)
			abssteer = -*cmdSteer;
		if (abssteer > 0) //带转向情况下，防抱死刹车
			if (_speed > 150)
				*cmdBrake = 0.5;
			else if (_speed > 100)
				*cmdBrake = 0.4;
			else
				*cmdBrake = 0.3;
		else {//不带转向情况下的防抱死刹车
			double brake_control = _speed * 0.01;//刹车力度判断
			if (brake_control > 1.5)
				*cmdBrake = 1.0;
			else if (brake_control > 1.4)
				*cmdBrake = 0.9;
			else if (brake_control > 1.3)
				*cmdBrake = 0.8;
			else if (brake_control > 1.2)
				*cmdBrake = 0.7;
			else if (brake_control > 1.1)
				*cmdBrake = 0.6;
			else if (brake_control > 1.0)
				*cmdBrake = 0.5;

			else
				*cmdBrake = 0.3;
		}
	}
	
	//更新档位
	updateGear(cmdGear);

	//窗口可视化
	 cls_visual.Fig2Y(1, -1, 1, 0, 500, 10, "Steer", *cmdSteer, "targetSpeed", targetSpeed,"*brake_control" , *cmdBrake);
	//cls_visual.Fig2Y(1, 0, 300, 0, 500, 10, "Target V", targetSpeed, "Curvature", minCruve, "Current V", _speed);
}
