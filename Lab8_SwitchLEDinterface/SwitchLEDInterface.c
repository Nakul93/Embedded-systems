// ***** 0. Documentation Section *****
// SwitchLEDInterface.c for Lab 8
// Runs on LM4F120/TM4C123
// Use simple programming structures in C to toggle an LED
// while a button is pressed and turn the LED on when the
// button is released.  This lab requires external hardware
// to be wired to the LaunchPad using the prototyping board.
// December 28, 2014
//      Jon Valvano and Ramesh Yerraballi

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****
unsigned long In;	// Input from PE0
// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Delay100ms(unsigned long time); // Delay100ms
void PortE_Init(void);

// ***** 3. Subroutines Section *****

// PE0, PB0, or PA2 connected to positive logic momentary switch using 10k ohm pull down resistor
// PE1, PB1, or PA3 connected to positive logic LED through 470 ohm current limiting resistor
// To avoid damaging your hardware, ensure that your circuits match the schematic
// shown in Lab8_artist.sch (PCB Artist schematic file) or 
// Lab8_artist.pdf (compatible with many various readers like Adobe Acrobat).
int main(void){ 
//**********************************************************************
// The following version tests input on PE0 and output on PE1
//**********************************************************************
  TExaS_Init(SW_PIN_PE0, LED_PIN_PE1);  // activate grader and set system clock to 80 MHz
  PortE_Init();
	
  EnableInterrupts();           // enable interrupts for the grader
  while(1){
    // body goes here
		//for(i=0;i<2666667;i++); // Delay for about 100 ms 100 ms * 80 MHZ/3
		Delay100ms(1);
		In = GPIO_PORTE_DATA_R&0x01; // read PE0 into Sw1
		if (In == 0x01) {
			GPIO_PORTE_DATA_R = GPIO_PORTE_DATA_R ^ 0x02;
		} 
		else {
			GPIO_PORTE_DATA_R |= 0x02;
		}
  }
  
}

void PortE_Init(void) {
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x10;	//1) activate clock for Port E
	delay = SYSCTL_RCGC2_R; // allow time for clock to start
	
	//GPIO_PORTF_LOCK_R = 0x4C4F434B; // 2) unlock GPIO Port F
  //GPIO_PORTF_CR_R = 0x1F; // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
	
	GPIO_PORTE_AMSEL_R &= ~0x03; // 3) disable analog on PE0 and PE1
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) PCTL GPIO on PF2 4
  GPIO_PORTE_DIR_R &= ~0x01; // 5) PE0 input(0)
  GPIO_PORTE_DIR_R |= 0x02; // 5) PE1 output(1)
  GPIO_PORTE_AFSEL_R &= ~0x03; // 6) disable alt funct on PE0 and PE1
  //GPIO_PORTE_PUR_R |= 0x10; // enable pull-up on PF0 and PF4 no needed
  GPIO_PORTE_DEN_R |= 0x03; // 7) enable digital I/O on PF2, 4
  GPIO_PORTE_DATA_R |= 0x02; // The system starts with the LED ON (make PE1 =1). 

}

void Delay100ms(unsigned long time){
	unsigned long i;
	while(time > 0){
		i = 1333333; // this number means 100ms
		while(i > 0){
			i = i - 1;
		}
		time = time - 1; // decrements every 100 ms
	}
}
