/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     This task controls the motors on the right side, specifically two DM4310s and one DM6215, all connected to the CAN1 bus.
	* 						 Looking down from the chassis, the DM4310 in the upper right corner has a transmit ID of 8 and a receive ID of 4.
	*						 The DM4310 in the lower right corner has a transmit ID of 6 and a receive ID of 3.
	*						 The right hub motor DM6215 has a transmit ID of 1 and a receive ID of 0.
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
	
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
 
float LQR_K_R[12]={ 
   -6.30722928759530	,-0.696842887297413	,-2.14750226828115	,-1.96028779204492	,3.16000121531755	,0.352046989571068,
    2.88563699845567	,0.321831107772386	,1.39325878380067	,1.18589341838622	,19.6627000823183	,0.927093573503157};
	
float Poly_Coefficient[12][4]={	{-254.842334407458	,178.727794135386,	-60.6919233036215	,-0.0855609578371201},
																{10.3190286075056	,-4.21383212156545	,-3.67395339191993	,0.0149909485593972},
																{-187.574261885913	,103.084830915149,	-20.0831869087043	,-0.766269544324169},
																{-121.714551237559	,69.1627701939264,	-15.0353152797156	,-0.821274420672333},
																{-279.503708963856	,238.311124550724,	-75.4693049740850	,10.8211229579575},
																{-0.747949466312385,	6.46656432845197	,-3.51917331980557,	0.802189665749108},
																{451.303971661993	,-197.027054972280	,20.5459194727689	,3.18497385998834},
																{32.4234051197324	,-16.4431941217205	,2.56514203767398	,0.229800316375223},
																{-149.488790812752,	127.122090472806	,-39.6954103984498	,5.36520535404774},
																{-220.550346615117	,150.142110973780	,-39.6803145871376	,4.84591726639935},
																{1983.04344474672	,-1086.94230997371	,211.953038244965	,5.06522197416514},
																{96.1185284211289	,-57.9583090999690,	12.8487400728423	-0.0549293400443370}};
vmc_leg_t right;

extern INS_t INS;
extern vmc_leg_t left;																
chassis_t chassis_move;
															
PidTypeDef LegR_Pid;//The length of the right leg (pd)	
PidTypeDef Tp_Pid;//Anti-splay compensation pd
PidTypeDef Turn_Pid;//Steering pd
PidTypeDef RollR_Pid;
uint32_t CHASSR_TIME=1;				
void ChassisR_task(void)
{
	while(INS.ins_flag==0)
	{//Waiting for acceleration to converge
	  osDelay(1);	
	}

  ChassisR_init(&chassis_move,&right,&LegR_Pid);//Initialize the IDs and control modes of the two right joint motors and the right hub motor, and initialize the leg
  Pensation_init(&Tp_Pid,&Turn_Pid);//Compensation PID initialization: anti-splay compensation, yaw compensation
	roll_pid_init(&RollR_Pid);
	while(1)
	{	
		chassisR_feedback_update(&chassis_move,&right,&INS);//Update data
		
	  chassisR_control_loop(&chassis_move,&right,&INS,LQR_K_R,&LegR_Pid);//Control calculations
   
		if(chassis_move.start_flag==1)	
		{
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1,chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f, chassis_move.wheel_motor[0].wheel_T);//Right hub motor
			osDelay(CHASSR_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1,chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//Right hub motor	
			osDelay(CHASSR_TIME);
		}
	
	}
}
void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr)
{
  const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	joint_motor_init(&chassis->joint_motor[0],6,MIT_MODE);//Tx id 6 
	joint_motor_init(&chassis->joint_motor[1],8,MIT_MODE);//Tx id 8
	
	wheel_motor_init(&chassis->wheel_motor[0],1,MIT_MODE);//Tx id 1
	
	VMC_init(vmc);//Set link lengths and initial positions
	
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//Leg length PID

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  osDelay(1);
	}

	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->wheel_motor[0].para.id,chassis->wheel_motor[0].mode);//Right hub motor
	  osDelay(1);
	}
}

void Pensation_init(PidTypeDef *Tp,PidTypeDef *turn)
{//Compensation PID initialization: anti-splay compensation, yaw compensation

	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);

}

void roll_pid_init(PidTypeDef *roll_pid)
{
	const static float roll[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	
	PID_init(roll_pid, PID_POSITION, roll, ROLL_PID_MAX_OUT,ROLL_PID_MAX_IOUT);
}
void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
  vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;
		
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[0];
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);
	
	if(ins->Pitch<(3.1415926f/6.0f)&&ins->Pitch>(-3.1415926f/6.0f))
	{//Determine if self-righting is complete based on pitch angle 
		chassis->recover_flag=0;
	}
}
uint32_t count_roll = 0; 
uint8_t right_flag=0;
extern uint8_t left_flag;
void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_right(vmcr,ins,((float)CHASSR_TIME)*3.0f/1000.0f);//Compute theta and d_theta for LQR, also right leg length L0; task period is 3*0.001 s
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );	
	}
		
	//chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw axis PID calculation
  chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//This calculation is more stable 

	chassis->leg_tp=PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);//Anti-splay PID calculation
	
	chassis->wheel_motor[0].wheel_T=(LQR_K[0]*(vmcr->theta-0.0f)
																	+LQR_K[1]*(vmcr->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_filter-chassis->x_set)
																	+LQR_K[3]*(chassis->v_filter-chassis->v_set)
																	+LQR_K[4]*(chassis->myPithR-0.0f)
																	+LQR_K[5]*(chassis->myPithGyroR-0.0f));
	
	//Right hip joint output torque				
	vmcr->Tp=(LQR_K[6]*(vmcr->theta-0.0f)
					+LQR_K[7]*(vmcr->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_filter-chassis->x_set)
					+LQR_K[9]*(chassis->v_filter-chassis->v_set)
					+LQR_K[10]*(chassis->myPithR-0.0f)
					+LQR_K[11]*(chassis->myPithGyroR-0.0f));
				
	chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T-chassis->turn_T;	//Hub motor output torque
	mySaturate(&chassis->wheel_motor[0].wheel_T,-1.0f,1.0f);	
	
	vmcr->Tp=vmcr->Tp+chassis->leg_tp;//Hip joint output torque


	chassis->now_roll_set = PID_Calc(&RollR_Pid,chassis->roll,chassis->roll_set);


	jump_loop_r(chassis,vmcr,leg);
		
	right_flag=ground_detectionR(vmcr,ins);//Right leg ground detection	
	 
	 if(chassis->recover_flag==0)		
	 {//Self-righting does not require ground detection	 
		if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
		{//When both legs are off the ground and the remote control is not controlling leg extension/retraction, it is considered off the ground
				chassis->wheel_motor[0].wheel_T=0.0f;
				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

				chassis->x_filter=0.0f;
				chassis->x_set=chassis->x_filter;
				chassis->turn_set=chassis->total_yaw;
				vmcr->Tp=vmcr->Tp+chassis->leg_tp;		
		}
		else
		{//Not off the ground
			vmcr->leg_flag=0;//Set to 0
			
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcr->Tp=0.0f;
	 }	 
	 
	mySaturate(&vmcr->F0,-150.0f,150.0f);//Saturation limit 
	
	VMC_calc_2(vmcr);//Calculate expected joint output torque

	//Rated torque saturation
  mySaturate(&vmcr->torque_set[1],-4.0f,4.0f);	
	mySaturate(&vmcr->torque_set[0],-4.0f,4.0f);		
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}


void jump_loop_r(chassis_t *chassis,vmc_leg_t *vmcr,PidTypeDef *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_r == 0)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.07f) ;//Feedforward + PD
			if(vmcr->L0<0.1f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 1;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 1;
			}
		}
		else if(chassis->jump_status_r == 1)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.4f) ;//Feedforward + PD
			if(vmcr->L0>0.16f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 2;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 2;
			}
		}
		else if(chassis->jump_status_r == 2)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//Feedforward + PD
			if(vmcr->L0<(chassis->leg_right_set+0.01f))
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 3;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 3;
			}
		}
		else
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//Feedforward + PD
		}

		if(chassis->jump_status_r == 3&&chassis->jump_status_l == 3)
		{
			chassis->jump_flag = 0;
			chassis->jump_time_r = 0;
			chassis->jump_status_r = 0;
			chassis->jump_time_l = 0;
			chassis->jump_status_l = 0;
		}
	}
	else
	{
		vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) - chassis->now_roll_set;//Feedforward + PD
	}

}

