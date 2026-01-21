#ifndef __PSTWO_TASK_H
#define __PSTWO_TASK_H

#include "main.h"
#include "chassisR_task.h"
#include "ins_task.h"

// Digital button constants (indexing into Data/MASK buffers)
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

// Analog stick value indices (used by PS2_AnologData)
#define PSS_RX 5               // Right stick X-axis value
#define PSS_RY 6               // Right stick Y-axis value
#define PSS_LX 7               // Left stick X-axis value
#define PSS_LY 8               // Left stick Y-axis value


typedef struct
{
    int16_t key;       // Current digital button bitmask
	int16_t last_key;   // Previous digital button bitmask
	
	int16_t lx;         // Left stick X-axis analog value
	int16_t ly;         // Left stick Y-axis analog value
	int16_t rx;         // Right stick X-axis analog value
	int16_t ry;         // Right stick Y-axis analog value
	
}ps2data_t;

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;

extern void PS2_data_read(ps2data_t *data);
extern void PS2_data_process(ps2data_t *data,chassis_t *chassis,float dt);
 
// Low-level PS2 interface helpers
uint8_t PS2_RedLight(void);                 // Check if controller is in analog (red light) mode
void PS2_ReadData(void);                    // Read one frame from controller
void PS2_Cmd(uint8_t CMD);                  // Send a single command byte to controller
uint8_t PS2_DataKey(void);                  // Get packed digital button state
uint8_t PS2_AnologData(uint8_t button);     // Get analog value for a given axis index
void PS2_ClearData(void);                   // Clear internal data buffers
void PS2_Vibration(uint8_t motor1, uint8_t motor2); // Control rumble motors (motor1 on/off 0xFF, motor2 strength 0x40~0xFF)

// Controller configuration helpers
void PS2_EnterConfing(void);                // Enter configuration mode
void PS2_TurnOnAnalogMode(void);            // Enable analog mode
void PS2_VibrationMode(void);               // Enable vibration mode
void PS2_ExitConfing(void);                 // Exit configuration mode
void PS2_SetInit(void);                     // Run full init sequence

extern void pstwo_task(void);
void jump_key (chassis_t *chassis,ps2data_t *data);



#endif



