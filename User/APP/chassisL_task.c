/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     This task controls the motors on the left side, specifically two DM4310s and one DM6215, all connected to the CAN2 bus.
	* 						 Looking down from the chassis, the DM4310 in the upper left corner has a transmit ID of 8 and a receive ID of 4.
	*						 The DM4310 in the lower left corner has a transmit ID of 6 and a receive ID of 3.
	*						 The left hub motor DM6215 has a transmit ID of 1 and a receive ID of 0.
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisL_task.h"
#include "fdcan.h"
#include "VMC_calc.h"

#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"

vmc_leg_t left;

float LQR_K_L[12]={ 
   -6.30722928759530	,-0.696842887297413	,-2.14750226828115	,-1.96028779204492	,3.16000121531755	,0.352046989571068,
    2.88563699845567	,0.321831107772386	,1.39325878380067	,1.18589341838622	,19.6627000823183	,0.927093573503157};

extern float Poly_Coefficient[12][4];

extern chassis_t chassis_move;
		
PidTypeDef LegL_Pid;
extern INS_t INS;

uint32_t CHASSL_TIME=1;				
void ChassisL_task(void)
{
  while(INS.ins_flag==0)
	{//Waiting for acceleration to converge
	  osDelay(1);	
	}	
  ChassisL_init(&chassis_move,&left,&LegL_Pid);//Initialize the IDs and control modes of the two left joint motors and the left hub motor, and initialize the leg
	
	while(1)
	{	
		chassisL_feedback_update(&chassis_move,&left,&INS);//Update data
		
		chassisL_control_loop(&chassis_move,&left,&INS,LQR_K_L,&LegL_Pid);//control calculations
   		
    if(chassis_move.start_flag==1)	
		{
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[1]);//left.torque_set[1]
			osDelay(CHASSL_TIME);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[0]);
			osDelay(CHASSL_TIME);
			mit_ctrl2(&hfdcan2,chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[1].wheel_T);//Left side hub motor
			osDelay(CHASSL_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
		  mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left.torque_set[1]
			osDelay(CHASSL_TIME);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
			osDelay(CHASSL_TIME);
			mit_ctrl2(&hfdcan2,chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//Left hub motor	
			osDelay(CHASSL_TIME);
		}
	}
}

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legl)
{
  const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	joint_motor_init(&chassis->joint_motor[2],6,MIT_MODE);// Tx id 6
	joint_motor_init(&chassis->joint_motor[3],8,MIT_MODE);// Tx id 8
	
	wheel_motor_init(&chassis->wheel_motor[1],1,MIT_MODE);// Tx id 1
	
	VMC_init(vmc);// Set link lengths
	
	PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);// Leg length PID

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan2,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan2,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	    enable_motor_mode(&hfdcan2,chassis->wheel_motor[1].para.id,chassis->wheel_motor[1].mode);// Left hub motor
	  osDelay(1);
	}
}

void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
  vmc->phi1=pi/2.0f+chassis->joint_motor[2].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[3].para.pos;
		
	chassis->myPithL=0.0f-ins->Pitch;
	chassis->myPithGyroL=0.0f-ins->Gyro[0];
	
}

extern uint8_t right_flag;
uint8_t left_flag;
void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_left(vmcl,ins,((float)CHASSL_TIME)*3.0f/1000.0f);// Compute theta and d_theta for LQR, also left leg length L0; task period is 3*0.001 s
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
	}
			
	chassis->wheel_motor[1].wheel_T=(LQR_K[0]*(vmcl->theta-0.0f)
																	+LQR_K[1]*(vmcl->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_set-chassis->x_filter)
																	+LQR_K[3]*(chassis->v_set-chassis->v_filter)
																	+LQR_K[4]*(chassis->myPithL-0.0f)
																	+LQR_K[5]*(chassis->myPithGyroL-0.0f));
	
	// Right hip joint output torque			
	vmcl->Tp=(LQR_K[6]*(vmcl->theta-0.0f)
					+LQR_K[7]*(vmcl->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_set-chassis->x_filter)
					+LQR_K[9]*(chassis->v_set-chassis->v_filter)
					+LQR_K[10]*(chassis->myPithL-0.0f)
					+LQR_K[11]*(chassis->myPithGyroL-0.0f));
	 		
	chassis->wheel_motor[1].wheel_T= chassis->wheel_motor[1].wheel_T-chassis->turn_T;	// Hub motor output torque
	mySaturate(&chassis->wheel_motor[1].wheel_T,-1.0f,1.0f);	
	
	vmcl->Tp=vmcl->Tp+chassis->leg_tp;// Hip joint output torque

//	vmcl->F0=13.0f/arm_cos_f32(vmcl->theta)+PID_Calc(leg,vmcl->L0,chassis->leg_left_set) + chassis->now_roll_set;// Feedforward + PD

	jump_loop_l(chassis,vmcl,leg); 	

	left_flag=ground_detectionL(vmcl,ins);// Left leg ground detection
	
	 if(chassis->recover_flag==0)	
	 {// No ground check needed when self-righting
		if(left_flag==1&&right_flag==1&&vmcl->leg_flag==0)
		{// Only treat as airborne when both legs leave ground and RC is not commanding leg extension
			chassis->wheel_motor[1].wheel_T=0.0f;
			vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);
			
			chassis->x_filter=0.0f;// Reset displacement
			chassis->x_set=chassis->x_filter;
			chassis->turn_set=chassis->total_yaw;
			vmcl->Tp=vmcl->Tp+chassis->leg_tp;		
		}
		else
		{// Not airborne
			vmcl->leg_flag=0;// Reset to 0			
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcl->Tp=0.0f;
	 }
	
	mySaturate(&vmcl->F0,-150.0f,150.0f);// Clamp 
	
	VMC_calc_2(vmcl);// Compute desired joint output torque
	
  // Rated torque
  mySaturate(&vmcl->torque_set[1],-4.0f,4.0f);	
	mySaturate(&vmcl->torque_set[0],-4.0f,4.0f);	
	
}
void jump_loop_l(chassis_t *chassis,vmc_leg_t *vmcl,PidTypeDef *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_l == 0)
		{
			vmcl->F0= Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,0.07f) ;// Feedforward + PD
			if(vmcl->L0<0.1f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 1)
		{
			vmcl->F0= Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,0.4f) ;// Feedforward + PD
			if(vmcl->L0>0.16f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 2)
		{
			vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_right_set) ;// Feedforward + PD
			if(vmcl->L0<(chassis->leg_right_set+0.01f))
			{
				chassis->jump_time_l++;
			}
		}
		else
		{
			vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_left_set) ;// Feedforward + PD
		}

	}
	else
	{
		vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_left_set) + chassis->now_roll_set;// Feedforward + PD
	}

}

