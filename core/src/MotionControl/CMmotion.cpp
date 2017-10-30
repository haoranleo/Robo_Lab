#include <iostream>
#include <math.h>
#include <stdio.h>
#include <CMmotion.h>
#include <utils.h>

const double FRAME_PERIOD = 1.0 / Param::Vision::FRAME_RATE;
//不知道用来干什么的一个函数，先注掉by zhy 2010.11.21
//float motion_time_1d(float dx,float vel0,float vel1,
//                            float max_vel,float max_accel,
//                            float &t_accel,float &t_cruise,float &t_decel){
//  float tmp,vmax;
//  const bool debug = true;
//  if(dx < 0){
//    dx = -dx;
//    vel0 = -vel0;
//    vel1 = -vel1;
//  }
//  if(debug){
//    printf("  d%f v%f-%f m%f a%f = ",
//           dx,vel0,vel1,max_vel,max_accel);
//  }
//  if(vel0 > max_vel) vel0 = max_vel;
//  if(vel1 > max_vel) vel1 = max_vel;
//
//  // stop
//  if(vel0 > vel1){
//    t_decel = (vel0 + vel1) / 2.0*dx;  //----->这是要干什么？
//    if(debug) printf("<%f> ",fabs(vel1 - vel0)/t_decel);
//    if(fabs(vel1 - vel0)/t_decel > max_accel){
//      t_accel = t_cruise = 0;
//      if(debug) printf("\n");
//      return(t_decel);
//    }
//  }//减速度不够
//
//  // calculate time spent at max velocity
//  tmp = 2*max_accel*dx + vel0*vel0 + vel1*vel1;
//  t_cruise = (tmp - 2*max_vel*max_vel) / (2*max_vel*max_accel);//匀速运动时间
//
//  if(t_cruise > 0){
//    vmax = max_vel;//能达到最大速度
//  }else{
//    vmax = sqrt((max_accel*dx + vel0*vel0 + vel1*vel1)/2);
//    t_cruise = 0;//达不到最大速度，加速到某一速度后减速
//  }
//
//  if(debug) printf("[am%f] ",vmax);
//
//  t_accel = max(vmax - vel0,(float)0.0) / max_accel;//加速时间
//  t_decel = fabs(vmax - vel1) / max_accel;//减速时间
//
//  if(debug){
//    printf("t(%f,%f,%f):%f\n",
//           t_accel,t_cruise,t_decel,t_accel+t_cruise+t_decel);
//  }
//
//  return(t_accel + t_cruise + t_decel);//总的时间
//}


 ////////////////////////////////////////////////////////////////////////////////////////////////////
 /// @fn	void compute_motion_1d(double x0, double v0, double v1, double a_max, double v_max,
 /// 		double a_factor, double &traj_accel, double &traj_time)
 ///
 /// @brief	一维状态下计算运动过程，给定初始速度和末速度，最大加速度和最大速度，返回加速时间和过程时间
 ///
 /// @author	Song Zhibai( Song Zhibai Zju.edu.cn)
 /// @date		2010-4-2
 ///
 /// @param	x0				初始位置
 /// @param	v0				初速度
 /// @param	v1				末速度
 /// @param	a_max			最大加速度
 /// @param	v_max			最大速度
 /// @param	a_factor		常数，影响加速度的因子
 /// @param	traj_accel		计算得加速度
 /// @param	traj_time		计算得加速时间
 ////////////////////////////////////////////////////////////////////////////////////////////////////

void compute_motion_1d(double x0, double v0, double v1,
    double a_max, double d_max, double v_max, double a_factor,
    double &traj_accel, double &traj_time)
{
    if (x0 == 0 && v0 == v1)
    { 
        traj_accel = 0;
        traj_time = 0;
        return; 
    }

    if(!finite(x0) || !finite(v0) || !finite(v1))
    {
        traj_accel = 0; 
        traj_time = 0;  
        return;
    }

    a_max /= a_factor;
    d_max /= a_factor;

    double accel_time_to_v1 = fabs(v1 - v0) / a_max; // 最大加速度加减速到末速度的时间
    double accel_dist_to_v1 = fabs((v1 + v0) / 2.0) * accel_time_to_v1; // 单一加速到末速度时的位移
    double decel_time_to_v1 = fabs(v0 - v1) / d_max; // 最大减速度减速到末速度的时间
    double decel_dist_to_v1 = fabs((v0 + v1) / 2.0) * decel_time_to_v1; // 单一减速到末速度时的位移

    // 这个时间很关键，设得较大则定位精度将大大降低 by qxz
    double period = 1.0 / 40.0; // 一段很小的时间，处理运动到目标点附近时加速度，稳定到点，防止超调

    v1 = _copysign(v1, -x0); // 坐标转换 将v1方向转换为位移的方向

    // 从x0运动到零点
    // 初速和目标点反向 或 初速大于目标速来不及减速到目标速
    // 全程减速
    if (v0 * x0 > 0 || (fabs(v0) > fabs(v1) && decel_dist_to_v1 > fabs(x0))) 
    {
        // 停下后到达的时间 + 停下所用时间
        double time_to_stop = fabs(v0) / d_max; // 停下时间
        double x_to_stop = v0 * v0 / (2.0 * d_max); // 停止时运动距离

        compute_motion_1d(x0 + _copysign(x_to_stop, v0), 0, v1, a_max * a_factor, d_max * a_factor,
            v_max, a_factor, traj_accel, traj_time); // 递归运算直到跳出这一条件
        traj_time += time_to_stop; // 路径规划时间加上去

        // 减速 
        if (traj_time < period)
            traj_accel = compute_stop(v0, d_max * a_factor); // 很快就要到了，计算一个合适的加速度
        else if (time_to_stop < period)
            // 这样处理使得加速度有一个由大到小的渐变
            traj_accel = time_to_stop / period * - _copysign(d_max * a_factor, v0) + (1.0 - time_to_stop / period) * traj_accel;
        else
            traj_accel = -_copysign(d_max * a_factor, v0);

        return;
    }

    // At this point we have two options.  We can maximally accelerate
    // and then maximally decelerate to hit the target.  Or we could
    // find a single acceleration that would reach the target with zero
    // velocity.  The later is useful when we are close to the target
    // where the former is less stable.

    // [[ OPTION 1
    // This computes the amount of time to accelerate before decelerating.
    double t_a, t_accel, t_decel;

    // 初速和目标点同向
    if (fabs(v0) > fabs(v1))
        // 初速度大于目标速，但可以减速到目标速
        // 先加速再减速
    {
        t_a = (sqrt((d_max * v0 * v0 + a_max * (v1 * v1 + 2 * d_max * fabs(x0)))
            / (a_max + d_max)) - fabs(v0)) / a_max; //初速度大于末速度，加速的时间

        if (t_a < 0.0)
            t_a = 0;
        t_accel = t_a;
        t_decel = ((fabs(v0) - fabs(v1)) + a_max * t_a) / d_max;
    } 

    else if (accel_dist_to_v1 > fabs(x0)) 
        // 初速度小于目标速，且不可加速到目标速
        // 全程加速
    {
        t_a = (sqrt(v0 * v0 + 2 * a_max * fabs(x0)) - fabs(v0)) / a_max;
        t_accel = t_a;
        t_decel = 0.0;//直接最大加速度加速到指定速度会超过目标点，加速不到指定末速度
    }

    else
        // 初速度小于目标速，且可以加速到目标速
        // 先加速再减速
    {
        t_a = (sqrt((d_max * v0 * v0 + a_max * (v1 * v1 + 2 * d_max * fabs(x0)))
            / (a_max + d_max)) - fabs(v0)) / a_max; //初速度大于末速度，加速的时间

        if (t_a < 0.0)
            t_a = 0;
        t_accel = t_a;
        t_decel = ((fabs(v0) - fabs(v1)) + a_max * t_a) / d_max;
    }
    // ]] OPTION 1

    // [[ OPTION 2
    double a_to_v1_at_x0 = (v0 * v0 - v1 * v1) / (2 * fabs(x0));//合适的加速度
    double t_to_v1_at_x0 =
        (-fabs(v0) + sqrt(v0 * v0 + 2 * fabs(a_to_v1_at_x0) * fabs(x0))) /
        fabs(a_to_v1_at_x0);//此时所需的时间

    // We follow OPTION 2 if t_a is less than a FRAME_PERIOD making it
    // difficult to transition to decelerating and stopping exactly.
    if (0 && a_to_v1_at_x0 < a_max && a_to_v1_at_x0 > 0.0 &&
        t_to_v1_at_x0 < 2.0 * FRAME_PERIOD && 0) 
    {
        // OPTION 2
        // Use option 1 time, even though we're not following it.
        traj_time = t_accel + t_decel;;

        // Target acceleration to stop at x0.
        traj_accel = -_copysign(a_to_v1_at_x0, v0);

        return;
    } 
    // ]] OPTION 2

    else
    {
        //printf("option 1");
        // OPTION 1
        // 加速时间 +　减速时间
        traj_time = t_accel + t_decel;

        // 计算所得车速可能超过车最大速度，会有一段匀速运动
        if (t_accel * a_max + fabs(v0) > v_max)
        {
            // 匀速运动的时间
            traj_time += pow(v_max - (a_max * t_accel + fabs(v0)), 2.0) / (2 * a_max) / v_max;
            traj_time += pow(v_max - (d_max * t_decel + fabs(v0)), 2.0) / (2 * d_max) / v_max;
        }

        // Accelerate (unless t_accel is less than FRAME_PERIOD, then set
        // acceleration to average acceleration over the period.)
        if (t_accel < period && t_decel == 0.0)
        {
            traj_accel = _copysign(a_max * a_factor, -x0);
        }
        else if (t_accel < period && t_decel > 0.0)
        {
            traj_accel = compute_stop(v0, d_max * a_factor);
        }
        else if (t_accel < period) 
        {
            traj_accel = _copysign((2.0 * t_accel / (period) - 1) * a_max * a_factor, v0);
        }
        else
        {
            traj_accel = _copysign(a_max * a_factor, -x0);
        }
    }
}


 ////////////////////////////////////////////////////////////////////////////////////////////////////
 /// @fn	void compute_motion_2d(CVector x0, CVector v0, CVector v1, double a_max, double v_max,
 /// 		double a_factor, CVector &traj_accel, double &time)
 ///
 /// @brief 在二维下计算运动过程，给定初始速度向量和最终速度向量，以及最大加速度，最大速度
 ///
 /// @author	Song Zhibai( Song Zhibai Zju.edu.cn)
 /// @date		2010-4-2
 ///
 /// @param	x0						initial coordinate
 /// @param	v0						initial velocities
 /// @param	v1						final velocities 
 /// @param	a_max					maxium of accelerate
 /// @param	v_max					maxium of velocities
 /// @param	a_factor				factor of accelerate
 /// @param [in,out]	traj_accel	the traj accel. 
 /// @param [in,out]	time		the time. 
 ////////////////////////////////////////////////////////////////////////////////////////////////////

void compute_motion_2d(CVector x0, CVector v0, CVector v1,
                    double a_max, double d_max, double v_max, 
                    double a_factor, CVector &traj_accel, double &time)
{
  double time_x;
  double time_y;
  double rotangle;
  double traj_accel_x;
  double traj_accel_y;
  if (v1.mod() > 0.0)
      rotangle = v1.dir();
  else
      rotangle = 0.0;

  x0 = x0.rotate(-rotangle);
  v0 = v0.rotate(-rotangle);
  v1 = v1.rotate(-rotangle); //坐标系转换，转换到末速度方向为x轴的坐标系中

  compute_motion_1d(x0.x(), v0.x(), v1.x(), a_max, d_max, v_max, a_factor,
		    traj_accel_x, time_x);
  compute_motion_1d(x0.y(), v0.y(), v1.y(), a_max, d_max, v_max, a_factor,
		    traj_accel_y, time_y);//两轴同样的最大速度、加速度独立考虑求两轴运动时间

  if (v1.mod() < 0.01)
  {
    double rx = time_x / hypot(time_x, time_y);
    double ry = time_y / hypot(time_x, time_y);
    double rx_a = rx;
	double rx_v = rx;
	//这样处理后能保证x轴、y轴运动同步？？？？？？疑问
	if (time_x < time_y * 1.5) 
    {
		rx_a = pow(time_x / (time_y * 1.5), 2.0);
		rx_v = time_x / (time_y * 1.5);
	}
    compute_motion_1d(x0.x(), v0.x(), v1.x(), a_max * rx_a, d_max * rx_a, v_max * rx_v, a_factor,
		      traj_accel_x, time_x);
    compute_motion_1d(x0.y(), v0.y(), v1.y(), a_max * ry, d_max * ry, v_max * ry, a_factor,
		      traj_accel_y, time_y);

	////////////////////////////////////////////////////////////////////////////
	///*
	//*	modified by yph at 2010-6-5： 进行二分迭代
	//*/

	////std::cout<<"Zero-Scale-before: x/y = "<<rx/ry<<std::endl;
	//rx = time_x / hypot(time_x, time_y);
	//ry = time_y / hypot(time_x, time_y);
	//const int MAX_ITERATE_NUM = 4;
	//for(int iterater = 0; iterater < MAX_ITERATE_NUM; iterater++)
	//{
	//	compute_motion_1d(x0.x(), v0.x(), v1.x(), a_max * rx, v_max * rx, a_factor,
	//			traj_accel_x, time_x);
	//	compute_motion_1d(x0.y(), v0.y(), v1.y(), a_max * ry, v_max * ry, a_factor,
	//			traj_accel_y, time_y);

	//	rx = time_x / hypot(time_x, time_y);
	//	ry = time_y / hypot(time_x, time_y);

	//	if( fabs((rx/ry)-1.0) < 0.1 )
	//		break;
	//}
	//std::cout<<"Zero-Scale-after: x/y = "<<rx/ry<<std::endl;

  } // if (v1.mod() < 0.01)
  else
  {
	// 非零速度到点
    if (time_x < time_y * 1.5) 
    {
      double rx_a = pow(time_x / (time_y * 1.5), 2.0);
      double rx_v = time_x / (time_y * 1.5);
      // x方向再次处理，最大速度最大加速度乘以一衰减因子，何用意？？？？？不考虑同步了？？？
      compute_motion_1d(x0.x(), v0.x(), v1.x(), a_max * rx_a, d_max * rx_a, v_max * rx_v, a_factor,
			traj_accel_x, time_x);
    }

	/*
	*	modified by yph at 2010-6-5： 进行二分迭代
	*/
	//double rx = time_x / hypot(time_x, time_y);
	//double ry = time_y / hypot(time_x, time_y);
	//const int MAX_ITERATE_NUM = 5;
	//for(int iterater = 0; iterater < MAX_ITERATE_NUM; iterater++)
	//{
	//	compute_motion_1d(x0.x(), v0.x(), v1.x(), a_max * rx, v_max * rx, a_factor,
	//		traj_accel_x, time_x);
	//	compute_motion_1d(x0.y(), v0.y(), v1.y(), a_max * ry, v_max * ry, a_factor,
	//		traj_accel_y, time_y);

	//	rx = time_x / hypot(time_x, time_y);
	//	ry = time_y / hypot(time_x, time_y);

	//	if( rx / ry >= 1.25 && rx / ry <= 20 )	//优先考虑y轴方向，保证y轴先到达
	//		break;
	//}
  }
  
  traj_accel = CVector(traj_accel_x,traj_accel_y);
  traj_accel = traj_accel.rotate(rotangle);
  time = std::max(time_x,time_y);
  //std::cout<<"time:"<<time<<"traj_accel:"<<traj_accel.mod()<<std::endl;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @fn	double compute_stop(double v, double max_a)
  ///
  /// @brief	Calculates the stop. 
  ///
  /// @author	Song Zhibai( Song Zhibai Zju.edu.cn)
  /// @date		2010-4-2
  ///
  /// @param	v		The v. 
  /// @param	max_a	The maximum a. 
  ///
  /// @return	The calculated stop. 
  ////////////////////////////////////////////////////////////////////////////////////////////////////

double compute_stop(double v, double max_a)
{
  if (fabs(v) > max_a * FRAME_PERIOD) return _copysign(max_a, -v); // 如果一帧内不能减速到0，就用最大加速度
  else return -v / FRAME_PERIOD; // 如果能减到0，加速度就用速度除以一帧的时间即可
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @fn	void goto_point_omni( const PlayerPoseT& start, const PlayerPoseT& final,
  /// 		const PlayerCapabilityT& capability, const double& accel_factor,
  /// 		const double& angle_accel_factor, PlayerPoseT& nextStep)
  ///
  /// @brief	给定初始坐标，结束坐标及小车性能，计算小车可能的轨迹，更新下一个位置的机器人状态
  ///
  /// @author	Song Zhibai( Song Zhibai Zju.edu.cn)
  /// @date		2010-4-2
  ///
  /// @param	start				The start. 
  /// @param	final				The final. 
  /// @param	capability			The capability. 
  /// @param	accel_factor		The accel factor. 
  /// @param	angle_accel_factor	The angle accel factor. 
  /// @param [in,out]	nextStep	the next step. 
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  void goto_point_omni( const PlayerPoseT& start,
	  const PlayerPoseT& final,
	  const PlayerCapabilityT& capability,
	  const double& accel_factor,
	  const double& angle_accel_factor,
	  PlayerPoseT& nextStep)
  {
	  CGeoPoint target_pos = final.Pos();
	  CVector x = start.Pos() - target_pos;
	  CVector v = start.Vel();
	  double ang = Utils::Normalize(start.Dir() - final.Dir());
	  double ang_v = start.RotVel();
      CVector target_vel = final.Vel();
	  double max_accel = capability.maxAccel;
      double max_decel = capability.maxDec;
	  double max_speed = capability.maxSpeed;
	  double max_angle_speed = capability.maxAngularSpeed;
	  double max_angle_accel = capability.maxAngularAccel;
      double max_angle_decel = capability.maxAngularDec;
	  CVector a;    
	  double ang_a, factor_a;
	  double time_a, time;
	  compute_motion_2d(x, v, target_vel, 
		  max_accel, 
          max_decel,
		  max_speed,
		 accel_factor,
		  a, time);
	  factor_a = 0.5 - 0.1;
	  //用迭代来迫近factor_a
	  do{
		  factor_a += 0.1;
		  compute_motion_1d(ang, ang_v, 0.0,
			  max_angle_accel*factor_a,
              max_angle_decel*factor_a,
			  max_angle_speed,
			 angle_accel_factor,
			  ang_a, time_a);
	  }while(factor_a<1.0 && time_a>time);
	  // printf("factor_a = %f\n",factor_a);

	  v = v + a * FRAME_PERIOD;
	  ang_v += ang_a * FRAME_PERIOD;
	  if (v.mod() > max_speed) 
		  v = v* max_speed/v.mod();
	  if (ang_v>max_angle_speed)
	  {
		  ang_v = max_angle_speed;
	  }
      else if (ang_v<-max_angle_speed)
	  {
		  ang_v = -max_angle_speed;
	  }
	  CGeoPoint next_pos = start.Pos()+Utils::Polar2Vector(v.mod()*FRAME_PERIOD,v.dir());
	  double next_angle = start.Dir()+ang_v*FRAME_PERIOD;
      double t =  std::max(time,time_a);
	  nextStep.SetValid(true);
	  nextStep.SetPos(next_pos);
	  nextStep.SetDir(next_angle);
	  nextStep.SetVel(v);
	  nextStep.SetRotVel(ang_v);
  }

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @fn	double expectedCMPathTime(const PlayerPoseT& start, const CGeoPoint& final,
/// 	double maxAccel, double maxVelocity, double accel_factor)
///
/// @brief	调用 compute_motion_2d
///
/// @author	Song Zhibai( Song Zhibai Zju.edu.cn)
/// @date	2010-4-2
///
/// @param	start			The start. 
/// @param	final			The final. 
/// @param	maxAccel		The maximum accel. 
/// @param	maxVelocity		The maximum velocity. 
/// @param	accel_factor	The accel factor. 
///
/// @return	. 
////////////////////////////////////////////////////////////////////////////////////////////////////

double expectedCMPathTime(const PlayerPoseT& start, const CGeoPoint& final, double maxAccel, double maxVelocity, double accel_factor)
{
	CGeoPoint target_pos = final;
	CVector x = start.Pos() - target_pos;
	CVector v = start.Vel();
	CVector target_vel = CVector(0,0);
	CVector a;  
	double time;
	compute_motion_2d(x, v, target_vel, 
		maxAccel, 
        maxAccel,
		maxVelocity,
		accel_factor,
		a, time);
    return time;
}
