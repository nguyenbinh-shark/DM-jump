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
	
	data->last_key=data->key;
  
	if(chassis->start_flag==1)
	{// Enabled
		chassis->target_v=((float)(data->ry-128))*(-0.008f);// Forward/backward from right stick Y
		slope_following(&chassis->target_v,&chassis->v_set,0.005f); // Rate-limit velocity

		chassis->x_set=chassis->x_set+chassis->v_set*dt;
		chassis->turn_set=chassis->turn_set+(data->rx-127)*(-0.00025f);// Turn from right stick X
			
		// Leg length and roll targets from left stick
		chassis->leg_set=chassis->leg_set+((float)(data->ly-128))*(-0.000015f);
		chassis->roll_target= ((float)(data->lx-127))*(0.0025f);

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
	}else if(chassis->start_flag==0)
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




