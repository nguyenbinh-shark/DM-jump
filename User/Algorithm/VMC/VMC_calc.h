#ifndef __VMC_CALC_H
#define __VMC_CALC_H

#include "main.h"
#include "INS_task.h"

#define pi 3.1415926f
#define LEG_PID_KP  700.0f
#define LEG_PID_KI  0.0f
#define LEG_PID_KD  2500.0f
#define LEG_PID_MAX_OUT  130.0f // 130 Nm (max torque output)
#define LEG_PID_MAX_IOUT 0.0f

typedef struct
{
	/* Fixed link lengths of the leg */
	float l5;// AE segment length (m)
	float l1;// meters
	float l2;// meters
	float l3;// meters
	float l4;// meters
	
	float XB,YB;// Point B coordinates
	float XD,YD;// Point D coordinates
	
	float XC,YC;// Point C coordinates
	float L0,phi0;// C radial distance and angle
	float alpha;
	float d_alpha;	
	
	float	lBD;// Distance BD
	
	float d_phi0;// Rate of phi0 at C
	float last_phi0;// Previous phi0 (for rate)

	float A0,B0,C0;// Intermediates
	float phi2,phi3;
	float phi1,phi4;
	
	float j11,j12,j21,j22;// Jacobian coefficients
	float torque_set[2];

	float F0;
	float Tp;
	
	float theta;
	float d_theta;// First derivative of theta
	float last_d_theta;
	float dd_theta;// Second derivative of theta
	
	float d_L0;// First derivative of L0
	float dd_L0;// Second derivative of L0
	float last_L0;
	float last_d_L0;
	
	float FN;// Normal force estimate
	
	uint8_t first_flag;
	uint8_t leg_flag;// Leg motion flag
} vmc_leg_t;

extern void VMC_init(vmc_leg_t *vmc);// Set default link lengths

extern void VMC_calc_1_right(vmc_leg_t *vmc,INS_t *ins,float dt);// Compute theta/d_theta (right leg) and L0
extern void VMC_calc_1_left(vmc_leg_t *vmc,INS_t *ins,float dt);
extern void VMC_calc_2(vmc_leg_t *vmc);// Compute joint torques from Cartesian wrench

extern uint8_t ground_detectionR(vmc_leg_t *vmc,INS_t *ins);// Ground contact detect (right)
extern uint8_t ground_detectionL(vmc_leg_t *vmc,INS_t *ins);// Ground contact detect (left)

extern float LQR_K_calc(float *coe,float len);
	
#endif




