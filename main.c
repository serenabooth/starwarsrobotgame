//////////////////////////////////////////////////////
// CS266 / CS189                                    //
// Final Project -- Darth Vader Edition             //
// Serena Booth & Lane Erickson                     //
//////////////////////////////////////////////////////

#include <p30f6014A.h>

#include "stdio.h"  //Necessary for sprint 
#include "string.h" //Necessary for string manipulation 
#include "stdlib.h" //Neccessary for itoa conversion 

#include "math.h"

#include "e_epuck_ports.h"
#include "e_init_port.h"
#include "motor_led/utility.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"
#include "a2d/e_prox.h"
#include "uart/e_uart_char.h"
#include "bluetooth/btcom.h"

#include "I2C/e_I2C_master_module.h"
#include "I2C/e_I2C_protocol.h"
#include "additional_functions_seas.h"
#include "motor_led/advance_one_timer/e_agenda.h"

#include "camera/fast_2_timer/e_poxxxx.h"
#include "camera/fast_2_timer/e_po6030k.h"


unsigned char sel; //the position of the selector switch

//Buffer for the camera image
static unsigned char buffer[300];

// camera set up 

//if we change the line of interest, we may need to re-calibrate colors. 
#define LINE_OF_INTEREST 200
#define cam_mode RGB_565_MODE				//Value defined in e_poxxxx.h (RGB_565_MODE, GREY_SCALE_MODE, YUV_MODE)
#define cam_width 80
#define cam_heigth 1 
#define cam_zoom 8 							//Fully zoomed out
#define buffer_size cam_width*cam_heigth*2	//Multiply by 1 or 2 depending on if grayscale or color is used.	

#define FULL_SPEED 800

int red(int i) 
{
	return ((i & 0xF8) >> 3);
}

int green(int i, int j) 
{
	return ((i & 0x07) << 3) | ((j & 0xE0)>>5);
}

int blue(int i) 
{
	return i & 0x1F;
}

/* 
Params: int r, int g, int b
Returns: 0 or 1. 1 if r, g, b fit DV range. 0 otherwise. 
*/
int is_darth(int r, int g, int b)
{
	return ( r < 27 && r > 15 && g < 5 && b < 5);
}

/* 
Params: int r, int g, int b
Returns: 0 or 1. 1 if r, g, b fit goal range. 0 otherwise. 
*/
int is_goal(int r, int g, int b)
{
	return ( r > 18 && r < 28 && g > 28 && g < 41 && b < 5);
}

/* 
Params: int r, int g, int b
Returns: 0 or 1. 1 if r, g, b fit allied range. 0 otherwise. 
*/
int is_allied(int r, int g, int b) //green-yellow
{
	return ( r > 10 && r < 20 && g > 35 && g < 45);
}

// TESTING FUNCTION
void camera_sees(void) 
{		 
	//Returns 0 if setup parameters for the camera are ok. Returns -1 if not.       
	if(0 != e_poxxxx_config_cam((ARRAY_WIDTH -cam_width*cam_zoom)/2,LINE_OF_INTEREST,cam_width*cam_zoom,cam_heigth*cam_zoom,cam_zoom,cam_zoom,cam_mode))	
	{
		e_set_led(0, 1);  //Turn on center diode when robot is considered from the front if setup FAILED.
		while(1);         //And then stay passive 
	}
	manual_camera_calibration();
	e_poxxxx_write_cam_registers(); //Initialization and changes to the setup of the camera.

	myWait(10);
	e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
	while(!e_poxxxx_is_img_ready());		//Wait for capture to complete

	int i;
	// loop over the length of the camera buffer
	for (i = 20; i < 21; i+=2)
	{
		myWait(100);
		int	red_r = red(buffer[i]);
		int green_r = green(buffer[i], buffer[i+1]);
		int blue_r = blue(buffer[i+1]);
		if (is_allied(red_r, green_r, blue_r))
		{
			btcomSendString("a"); 
		}
		else if (is_darth(red_r, green_r, blue_r))
		{
			btcomSendString("d"); 
		}
		else if (is_goal(red_r, green_r, blue_r))
		{
			btcomSendString("g"); 
		}
		else 
		{
			btcomSendString("_"); 
		}
		btcomSendInt(red_r);
		btcomSendString(" ");  
		btcomSendInt(green_r);
		btcomSendString(" ");  
		btcomSendInt(blue_r);
		btcomSendString(" \r\n\r\r\n\r");  
	}
}

/*
Code modified from PS3 solution. 
Returns -1 for camera set up failure. 
Returns 0 for Alliance member in sight.
Returns 1 for captured alliance member.
Returns 2 for no Alliance member in sight.  
*/
int sees_alliance() 
{		 
	if(0 != e_poxxxx_config_cam((ARRAY_WIDTH -cam_width*cam_zoom)/2,LINE_OF_INTEREST,cam_width*cam_zoom,cam_heigth*cam_zoom,cam_zoom,cam_zoom,cam_mode))	
	{
		e_set_led(0, 1);  //Turn on center diode when robot is considered from the front if setup FAILED.
		while(1);         //And then stay passive 
		return -1; 
	}
	manual_camera_calibration();
	e_poxxxx_write_cam_registers(); //Initialization and changes to the setup of the camera.

	myWait(10);
	e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
	while(!e_poxxxx_is_img_ready());		//Wait for capture to complete

	// we use these values to compute the colors in each of pixels 1 and 2, checking for consistency
	int r = 0;
	int g = 0; 
	int b = 0;

	int i = 0; 
			
	// we use these values to calculate the size of the alliance member  
	int largest_start = -1;
	int largest_fin = -1;
	int temp_start = -1;
	int temp_fin = -1;
			
	// in sequence is used to check for broken pixels
	int in_seq = 0;

	// loop over the length of the camera buffer
	for (i = 0; i < 160; i+=2)
	{	
		// read in color data for pixels i through i+1  and i+2 through i+3
		r = red(buffer[i]);
		g = green(buffer[i], buffer[i+1]);
		b = blue(buffer[i+1]);
		// compute size of red blob 
		if (in_seq) 
		{	
			if (is_allied(r,g,b))
			{
				temp_fin += 2;
			}
			else if ((temp_fin-temp_start > 2) && (temp_fin-temp_start > largest_fin-largest_start))
			{
				largest_fin = temp_fin;
				largest_start = temp_start;
			}
			else 
			{
				in_seq = 0;
			}
		}
		else 
		{
			if (is_allied(r,g,b)) 
			{
				temp_start = i;
				temp_fin = i + 2;
				in_seq = 1;
			}	
		}
	}
	// 132 is about perfect, 80 is just outside of gate
	int size = largest_fin-largest_start;
	// robot's right is < 80, robot's left is > 80
	int position = (largest_fin+largest_start)/2;

	btcomSendInt(size);
	btcomSendString(" ");
	btcomSendInt(position);
	btcomSendString("\r\n");
			
	//conditional for speed, stops if Alliance member is killed.
	int speed = 80;
	if (size > 140)
	{
		e_set_speed_left(0);
		e_set_speed_right(0);	
		btcomSendInt(2);
		return 1;
	}
	// set speeds based on position of Alliance member
	if (size != 0)
	{
		if (position < 76)
		{
			e_set_speed_left(10*speed);
			e_set_speed_right(7*speed);	
		}
		else if (position > 84)
		{
			e_set_speed_left(7*speed);
			e_set_speed_right(10*speed);
		}
		else
		{
			e_set_speed_right(10*speed);
			e_set_speed_left(10*speed);
		}
		
		btcomSendInt(0);
		return 0;
	}
	else 	// No Alliance member in sight. 
	{
		btcomSendInt(1);
		return 2; 
	}
}

/*
Code modified from PS3 solution. 
Returns -1 for camera set up failure. 
Returns 0 if we cannot see Darth 
Returns size of darth if we can see Darth.  
*/
int sees_darth(void)
{		 
	if(0 != e_poxxxx_config_cam((ARRAY_WIDTH -cam_width*cam_zoom)/2,LINE_OF_INTEREST,cam_width*cam_zoom,cam_heigth*cam_zoom,cam_zoom,cam_zoom,cam_mode))	
	{
		e_set_led(0, 1);  //Turn on center diode when robot is considered from the front if setup FAILED.
		while(1);         //And then stay passive 
		return -1; 
	}
	manual_camera_calibration();
	e_poxxxx_write_cam_registers(); //Initialization and changes to the setup of the camera.

	myWait(10);
	e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
	while(!e_poxxxx_is_img_ready());		//Wait for capture to complete

	// we use these values to compute the colors in each of pixels 1 and 2, checking for consistency
	int r = 0;
	int g = 0; 
	int b = 0;

	int i = 0; 
			
	// we use these values to calculate the size of the alliance member  
	int largest_start = -1;
	int largest_fin = -1;
	int temp_start = -1;
	int temp_fin = -1;
			
	// in sequence is used to check for broken pixels
	int in_seq = 0;

	// loop over the length of the camera buffer
	for (i = 0; i < 160; i+=2)
	{	
		// read in color data for pixels i through i+1  and i+2 through i+3
		r = red(buffer[i]);
		g = green(buffer[i], buffer[i+1]);
		b = blue(buffer[i+1]);
		// compute size of red blob 
		if (in_seq) 
		{	
			if (is_darth(r,g,b))
			{
				temp_fin += 2;
			}
			else if ((temp_fin-temp_start > 2) && (temp_fin-temp_start > largest_fin-largest_start))
			{
				largest_fin = temp_fin;
				largest_start = temp_start;
			}
			else 
			{
				in_seq = 0;
			}
		}
		else 
		{
			if (is_darth(r,g,b)) 
			{
				temp_start = i;
				temp_fin = i + 2;
				in_seq = 1;
			}	
		}
	}
	// 132 is about perfect, 80 is just outside of gate
	int size = largest_fin-largest_start;
	if (size > 0)
		return size; 
	else
		return 0; 

}


int main(void)
{	
	myWait(500);
	e_init_port();    //Configure port pins
	e_init_motors();
	e_init_uart1();   //Initialize UART to 115200 Kbit
	e_i2cp_init();    //I2C bus for the camera

	e_init_prox();	  //Initialize proximity sensors

	e_poxxxx_init_cam(); 					//Located in e_common.c 

	if(RCONbits.POR) {	//Reset if necessary
		RCONbits.POR=0;
		RESET();
	}

	sel = getselector();			//Read position of selector switch, refer to utility.c
	
	
	if(sel==0)						//During this class, sel==0 should always do nothing. This will be the programming mode.
	{
		while(1) NOP();
	}
	if(sel==1)						//Darth Vader Routine 
	{
		while(1)
		{
			myWait(100);
			if(sees_alliance() == 2) 
			{
				e_set_speed_right(FULL_SPEED/4);
				e_set_speed_left(-FULL_SPEED/4);
				myWait(20); 
			} 
		}
	}
	else if (sel == 2) // run towards goal until seeing darth, then run backwards to safe spot. 
	{

		int d = 0;
		
		/* 
		Returns 0 if darth vader is not getting closer
		Returns 1 otherwise
		*/
		int darth_getting_closer(int darth_prox)
		{
			myWait(100); 
			if (sees_darth() > darth_prox)
				return 1; 
			else
				return 0; 
		}

		while (1) 
		{
			int chk_darth = sees_darth(); 
			darth_getting_closer(chk_darth); 
			// DARTH is visible and getting closer
			if (chk_darth > 0 &&  darth_getting_closer(chk_darth))
			{
				// run back to safe location
				// pause for several seconds 
				// wrap around 
				// run to goal
			}
			// DARTH is not visible or is not getting closer. 
			else
			{
				// run towards goal with dead reckoning
				setSpeeds(800,800);
				myWait(40); 
			}
		}

	
	}
	else if (sel == 3) 
	{
			setSpeeds(-710,-800);
			myWait(4150);

			setSpeeds(0,0);	
			myWait(5000);
			setSpeeds(710,800);
			myWait(4150);
			
			setSpeeds(0,0);		
	}

	else if (sel == 4) 
	{
		moveForward(400,800);
		turn(-50,800);
		while (1) {
		setSpeeds(800,800);
		}
	}
	else if (sel == 5) 
	{
		moveForward(400,800);
		turn(-54,800);
		while (1) {
		setSpeeds(800,800);
		}
	}
	else							//None of those selected
	{							
		while(1) NOP();
	}
}
