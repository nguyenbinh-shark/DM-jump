/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     
  *           Obtain the vehicle velocity and position estimation based on inertial navigation linear acceleration in the navigation frame
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "observe_task.h"
#include "kalman_filter.h"
#include "cmsis_os.h"
#include "app_uart.h"

KalmanFilter_t vaEstimateKF;	   // Velocity and position estimation Kalman filter structure

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // State transition matrix, sampling time is 0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // Initial estimation error covariance

float vaEstimateKF_Q[4] = {0.5f, 0.0f, 
                           0.0f, 0.5f};    // Initial process noise covariance

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  100.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// Measurement matrix H is identity
														 															 
extern INS_t INS;		
extern chassis_t chassis_move;																 															 
																 
extern vmc_leg_t right;			
extern vmc_leg_t left;	

float vel_acc[2]; 
uint32_t OBSERVE_TIME=3;// Sampling time 3ms	
static uint32_t feedback_counter = 0;  // Counter for feedback rate limiting
#define FEEDBACK_DIVIDER 10  // Send feedback every 10 cycles = 30ms = ~33Hz
														 
void 	Observe_task(void)
{
	while(INS.ins_flag==0)
	{// Waiting for velocity to stabilize
	  osDelay(1);	
	}
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;
		
	xvEstimateKF_Init(&vaEstimateKF);
	
  while(1)
	{  
		wr= -chassis_move.wheel_motor[0].para.vel-INS.Gyro[0]+right.d_alpha;// Right wheel angular velocity considering chassis rotation and right leg rotation
		vrb=wr*0.0603f+right.L0*right.d_theta*arm_cos_f32(right.theta)+right.d_L0*arm_sin_f32(right.theta);// Right wheel linear velocity in body frame
		
		wl= -chassis_move.wheel_motor[1].para.vel+INS.Gyro[0]+left.d_alpha;// Left wheel angular velocity considering chassis rotation and left leg rotation
		vlb=wl*0.0603f+left.L0*left.d_theta*arm_cos_f32(left.theta)+left.d_L0*arm_sin_f32(left.theta);// Left wheel linear velocity in body frame
		
		aver_v=(vrb-vlb)/2.0f;// Average velocity
    xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_n[1],aver_v);
		
		// Original method to calculate v_filter and x_filter should be 0
		chassis_move.v_filter=vel_acc[0];// Obtained linear velocity in navigation frame
		chassis_move.x_filter=chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		
		// Send feedback to Python at ~33Hz (every 30ms)
		feedback_counter++;
		if (feedback_counter >= FEEDBACK_DIVIDER)
		{
			//uart_send_feedback(&chassis_move, &INS);
			feedback_counter = 0;
		}
		
	// Calculate linear velocity based on wheel angular velocity, used for comparison with above velocity
	//chassis_move.v_filter=(chassis_move.wheel_motor[0].para.vel-chassis_move.wheel_motor[1].para.vel)*(-0.0603f)/2.0f;//0.0603 is wheel radius, used to calculate linear velocity from angular velocity, wheel radius is obtained by measurement, used for comparison with above velocity
	//chassis_move.x_filter=chassis_move.x_filter+chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		
		osDelay(OBSERVE_TIME);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// State dimension 2, no control input, measurement dimension 2
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    // Update measurement vector with new values
    EstimateKF->MeasuredVector[0] =	vel;// Linear velocity
    EstimateKF->MeasuredVector[1] = acc;// Linear acceleration
    		
    // Perform Kalman filter update
    Kalman_Filter_Update(EstimateKF);

    // Retrieve filtered values
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}


