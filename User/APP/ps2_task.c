/**
	*********************************************************************
	* @file      ps2_task.c/h
	* @brief     PS2 controller task: read raw PS2 frame and map sticks/buttons to chassis targets
	*            Converts stick inputs to forward velocity, turning rate, body pitch/roll, and leg length.
	* @note
	* @history
	*
	@verbatim
	==============================================================================

	==============================================================================
	@endverbatim
	*********************************************************************
	*/
	
#include "ps2_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "tim.h"   // d? d�ng htim12
#include "bsp_PWM.h" // For buzzer control

ps2data_t ps2data;

uint16_t Handkey;	// Stores decoded digital button bits (0 = pressed)
uint8_t Comd[2]={0x01,0x42};	// Start command sequence for PS2
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // Buffer for raw frame data
uint16_t MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	// Mask map for button indices


extern chassis_t chassis_move;
extern INS_t INS;
uint32_t PS2_TIME=10;// PS2 poll interval in ms
void pstwo_task(void)
{		
	 PS2_SetInit();
	   while(1)
	 {
		 if(Data[1]!=0x73)
		 {
		  PS2_SetInit();
		 }

	   PS2_data_read(&ps2data);// Fetch one frame
		 
       // Always process PS2 control data
       PS2_data_process(&ps2data,&chassis_move,(float)PS2_TIME/1000.0f);// Map input to chassis targets
		 
	   osDelay(PS2_TIME);
	 }
}

// Send one command byte to controller and read response
void PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);                   //���һλ����λ DO_H;
		}
		else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);//DO_L

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);                        //ʱ������
		DWT_Delay(0.000005f);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
		DWT_Delay(0.000005f);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
			Data[1] = ref|Data[1];
	}
	DWT_Delay(0.000016f); 
}

/**************************************************************************
Function: Read controller states (buttons + sticks) into ps2data_t
Input   : data pointer
Output  : none
**************************************************************************/	
uint8_t reve_flag=0;

void PS2_data_read(ps2data_t *data)
{
  // Decode digital buttons
	data->key=PS2_DataKey();

  // Read analog sticks
	data->lx=PS2_AnologData(PSS_LX);
	data->ly=PS2_AnologData(PSS_LY);
	data->rx=PS2_AnologData(PSS_RX);
	data->ry=PS2_AnologData(PSS_RY);

	if((data->ry<=255&&data->ry>192)||(data->ry<64&&data->ry>=0))
	{
	  data->rx=127;
	}
	if((data->rx<=255&&data->rx>192)||(data->rx<64&&data->rx>=0))
	{
	  data->ry=128;
	}
}

extern vmc_leg_t right;			
extern vmc_leg_t left;	
float acc_test =0.005f;
void PS2_data_process(ps2data_t *data,chassis_t *chassis,float dt)
{
	// SELECT button toggles PS2 control mode (0: PS2 enabled, 1: PS2 disabled/UART reserved)
	PS2_mode_switch(data, chassis);
		chassis->start_flag=1;
	if(chassis->control_mode != 0)
	{
		// Ignore stick input and hold current pose/heading to avoid unintended yaw drift.
		chassis->target_v = 0.0f;
		chassis->v_set = 0.0f;
		chassis->x_set = chassis->x_filter;
		chassis->turn_set = chassis->total_yaw;
		chassis->roll_target = 0.0f;
		slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);

		data->last_key = data->key;
		return;
	}

	if(data->last_key!=4&&data->key==4&&chassis->start_flag==0) 
	{
		// Start button rising edge: arm start_flag
		chassis->start_flag=1;
		if(chassis->recover_flag==0
			&&((chassis->myPithR<((-3.1415926f)/4.0f)&&chassis->myPithR>((-3.1415926f)/2.0f))
		  ||(chassis->myPithR>(3.1415926f/4.0f)&&chassis->myPithR<(3.1415926f/2.0f))))
		{
		  chassis->recover_flag=1;// Needs recovery
		}
	}
	else if(data->last_key!=4&&data->key==4&&chassis->start_flag==1) 
	{
		// Start button rising edge: disarm start_flag
		chassis->start_flag=0;
		chassis->recover_flag=0;
	}

	static float speed_mult = 1.0f;
	static uint8_t speed_level = 1;
	
	static uint8_t beep_count = 0;
	static uint8_t beep_state = 0;
	static uint32_t level_beep_timer = 0;
	
	if(data->last_key!=9 && data->key==9) // L2 rising edge
	{
		speed_level++;
		if(speed_level > 2) speed_level = 0;
		
		if(speed_level == 0) speed_mult = 0.5f;
		else if(speed_level == 1) speed_mult = 1.0f;
		else if(speed_level == 2) speed_mult = 1.5f;
		
		// Báo còi: 1 tiếng cho 0.5x, 2 tiếng cho 1.0x, 3 tiếng cho 1.5x
		beep_count = speed_level + 1; 
		beep_state = 1; // start beeping
		level_beep_timer = HAL_GetTick();
		TIM_Set_PWM(&htim12, TIM_CHANNEL_2, 500);
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	}

	// State machine xử lý còi kêu không chặn (non-blocking)
	if(beep_state > 0)
	{
		uint32_t now = HAL_GetTick();
		if(beep_state == 1 && (now - level_beep_timer) >= 100) // 100ms ON
		{
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			level_beep_timer = now;
			beep_count--;
			
			if(beep_count > 0) beep_state = 2; // Pause between beeps
			else beep_state = 0; // Done
		}
		else if(beep_state == 2 && (now - level_beep_timer) >= 100) // 100ms OFF
		{
			TIM_Set_PWM(&htim12, TIM_CHANNEL_2, 500);
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
			level_beep_timer = now;
			beep_state = 1;
		}
	}

	data->last_key=data->key;

	if(chassis->start_flag==1)
	{// Enabled
		if (data->key == 11) // PSB_L1
		{
			chassis->target_v = 0.5f;
		}
		else
		{
			chassis->target_v=((float)(data->ry-128))*(-0.008f)*speed_mult;// Forward/backward from right stick Y
		}
		slope_following(&chassis->target_v,&chassis->v_set,0.005f); // Rate-limit velocity

		chassis->x_set=chassis->x_set+chassis->v_set*dt;
		chassis->turn_set=chassis->turn_set+(data->rx-127)*(-0.00025f);// Turn from right stick X

		// Leg length from left stick Y
		if(data->ly > 125 || data->ly < 130) // Deadzone around center (128)
		{
		chassis->leg_set=chassis->leg_set+((float)(data->ly-128))*(-0.000015f);
		}
		// Yaw control from left stick X (with deadzone for center offset at 128)
		if(data->lx > 120 || data->lx < 135) // Deadzone around center (128)
		{
			chassis->turn_set=chassis->turn_set+((float)(data->lx-128))*(-0.00025f);
		}

		// Roll control from L PAD buttons
		if(data->key == 8) // PSB_PAD_LEFT
		{
			chassis->roll_target = 0.40f; // Roll left
		}
		else if(data->key == 6) // PSB_PAD_RIGHT
		{
			chassis->roll_target = -0.40f; // Roll right
		}
		else
		{
			chassis->roll_target = 0.0f; // Center roll when no button pressed
		}

		slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);

		jump_key(chassis,data);

		chassis->leg_left_set = chassis->leg_set;
		chassis->leg_right_set = chassis->leg_set;

		mySaturate(&chassis->leg_left_set,0.060f,0.21f);// Limit leg length range
		mySaturate(&chassis->leg_right_set,0.060f,0.21f);

		if(fabsf(chassis->last_leg_left_set-chassis->leg_left_set)>0.0001f || fabsf(chassis->last_leg_right_set-chassis->leg_right_set)>0.0001f)
		{// Stick changed leg setpoint -> flag leg motion
			right.leg_flag=1;	// Flag to allow leg kinematics update
      left.leg_flag=1;	 			
		}
		chassis->last_leg_set=chassis->leg_set;
		chassis->last_leg_left_set=chassis->leg_left_set;
		chassis->last_leg_right_set=chassis->leg_right_set;
		}
	else if(chassis->start_flag==0)
	{// Disabled: hold pose
	  chassis->v_set=0.0f;// zero velocity
	  chassis->x_set=chassis->x_filter;// hold position
	  chassis->turn_set=chassis->total_yaw;// hold heading
	  chassis->leg_set=0.08f;// default leg length
		}
	}
void jump_key (chassis_t *chassis,ps2data_t *data)
{
	if(data->key == 12)
	{
		if(++chassis->count_key>10)
		{
			if(chassis->jump_flag == 0)
			{
				chassis->jump_flag = 1;
				chassis->jump_leg = chassis->leg_set;
			}
		}
	}
	else
	{
		chassis->count_key = 0;
	}
}

/**
 * @brief Handle Triangle button press to toggle control mode with buzzer feedback
 * @param data: PS2 data structure
 * @param chassis: chassis structure
 * @note Button 13 (PSB_GREEN / Triangle) toggles between PS2 mode (0) and UART mode (1)
 *       Plays different tones for each mode: short beep for PS2, double beep for UART
 */
static uint8_t last_mode_btn_state = 0;
static uint32_t last_mode_switch_time = 0;
static uint32_t beep_timer = 0;
static uint8_t beep_stage = 0;

#define PS2_TOGGLE_KEY PSB_GREEN

void PS2_mode_switch(ps2data_t *data, chassis_t *chassis)
{
	// Handle buzzer beeping for mode feedback
	uint32_t now = HAL_GetTick();
	
	if(beep_stage > 0)
	{
		if(beep_stage == 1 && (now - beep_timer) >= 100) // Short beep ON (100ms)
		{
			// Turn off buzzer
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			beep_timer = now;
			
			if(chassis->control_mode == 0) // PS2 mode = single beep
			{
				beep_stage = 0; // Done
			}
			else // UART mode = double beep
			{
				beep_stage = 2; // Wait before second beep
			}
		}
		else if(beep_stage == 2 && (now - beep_timer) >= 100) // Pause between beeps (100ms)
		{
			// Second beep ON
			TIM_Set_PWM(&htim12, TIM_CHANNEL_2, 500); // 2kHz tone
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
			beep_timer = now;
			beep_stage = 3;
		}
		else if(beep_stage == 3 && (now - beep_timer) >= 100) // Second beep duration (100ms)
		{
			// Turn off buzzer
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
			beep_stage = 0; // Done
		}
	}
	
	// Detect toggle button rising edge with 500ms debounce
	if(last_mode_btn_state == 0 && data->key == PS2_TOGGLE_KEY && (now - last_mode_switch_time > 500))
	{
		last_mode_switch_time = now;

		// Toggle control mode
		chassis->control_mode = (chassis->control_mode == 0) ? 1 : 0;
		
		// Start beep sequence
		TIM_Set_PWM(&htim12, TIM_CHANNEL_2, 500); // 2kHz tone (ARR=1000, CCR=500)
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
		beep_timer = HAL_GetTick();
		beep_stage = 1;
	}
	
	// Update state for next call
	last_mode_btn_state = (data->key == PS2_TOGGLE_KEY) ? 1 : 0;
}

// Check whether controller is in analog mode: 0x41 = green light (digital), 0x73 = red light (analog)
// return 0 -> analog mode, 1 -> digital mode
uint8_t PS2_RedLight(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
	PS2_Cmd(Comd[0]);  //start command
	PS2_Cmd(Comd[1]);  //rumble command
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
	if( Data[1] == 0X73)   return 0 ;
	else return 1;
}
// Read one full PS2 frame into Data[]
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	PS2_Cmd(Comd[0]);  //start command
	PS2_Cmd(Comd[1]);  //rumble command
	for(byte=2;byte<9;byte++)          //start data read
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);//CLK_H
			DWT_Delay(0.000005f);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);//CLK_L
			DWT_Delay(0.000005f);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);//CLK_H
		      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))//DI
		      Data[byte] = ref|Data[byte];
		}
        DWT_Delay(0.000016f); 
	}
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
}
// Decode PS2 frame into button index (1..16). Only one button considered here.
// Pressed bit is 0, unpressed is 1.
uint8_t PS2_DataKey()
{
	uint8_t index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //16-bit button state, 0 = pressed, 1 = not pressed
	for(index=0;index<16;index++)
	{
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //no button pressed
}
// Get analog value of a stick axis (0~256)
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button];
}
// Clear Data[] buffer
void PS2_ClearData()
{
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: Control rumble motors
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1: right small motor (0x00 off, 0xFF on)
	   motor2: left big motor strength 0x40~0xFF (higher = stronger)
******************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	DWT_Delay(0.000016f); 
  PS2_Cmd(0x01);  //start command
	PS2_Cmd(0x42);  //rumble command
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f);  
}
//short poll
void PS2_ShortPoll(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 	
}
// Enter config mode
void PS2_EnterConfing(void)
{
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 
}
// Set analog mode
void PS2_TurnOnAnalogMode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  �������÷���ģʽ
	PS2_Cmd(0x03); //Ox03�������ã�������ͨ��������MODE������ģʽ��
				   //0xEE�������������ã���ͨ��������MODE������ģʽ��
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 
}
// Enable vibration
void PS2_VibrationMode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 
}
// Exit config mode
void PS2_ExitConfing(void)
{
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 
}
// Full initialization sequence
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//��������ģʽ
	PS2_TurnOnAnalogMode();	//�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	//PS2_VibrationMode();	//������ģʽ
	PS2_ExitConfing();		//��ɲ���������
}




