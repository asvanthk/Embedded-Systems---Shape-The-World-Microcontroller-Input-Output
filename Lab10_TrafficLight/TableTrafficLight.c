// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void portEinit(void);
void portFinit(void);
void portBinit(void);

// ***** 3. Subroutines Section *****

void PLLinit(void)
	{
  // 0) Use RCC2
  SYSCTL_RCC2_R |=  0x80000000;  // USERCC2
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |=  0x00000800;  // BYPASS2, PLL bypass
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R = (SYSCTL_RCC_R &~0x000007C0)+ 0x00000540;   // clear XTAL field, bits 10-6 // 10101, configure for 16 MHz crystal
                   
  SYSCTL_RCC2_R &= ~0x00000070;  // configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~0x00002000;
  // 4) set the desired system divider
  SYSCTL_RCC2_R |= 0x40000000;   // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000) + (4<<22);  // clear system clock divider
                        // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~0x00000800;
}

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = 0x00FFFFFF;        // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it             
  NVIC_ST_CTRL_R = 0x00000005;          // enable SysTick with core clock
}


void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  
  NVIC_ST_CURRENT_R = 0;       
  while((NVIC_ST_CTRL_R&0x00010000)==0){ 
  }
}

void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }

}
struct State {
  unsigned long t_out;
	unsigned long w_out;
  unsigned long Time;  
  unsigned long Next[12];
}; 

typedef const struct State STyp;
//define states
#define goW    0
#define waitW  1
#define goS    2
#define waitS  3
#define stopWS 4
#define walk   5
#define flash1 6
#define flash2 7
#define flash3 8
#define flash4 9
#define flash5 10
#define flash6 11


STyp FSM[12]={
 {0x0C,0x02,30,{goW,goW,waitW,waitW,waitW,waitW,waitW,waitW}},//goW 
 {0x14,0x02,30,{goS,goS,goS,goS,stopWS,stopWS,stopWS,goS}}, //waitW
 {0x21,0x02,30,{goS,waitS,goS,waitS,waitS,waitS,waitS,waitS}},//goS
 {0x22,0x02,30,{goW,goW,goW,goW,stopWS,stopWS,stopWS,stopWS}},//waitS
 {0x24,0x02,30,{goW,goS,goW,goS,walk,walk,walk,walk}},//stopWS
 {0x24,0x08,30,{walk,flash1,flash1,flash1,walk,flash1,flash1,flash1}}, //walk
 {0x24,0x02,30,{flash2,flash2,flash2,flash2,flash2,flash2,flash2,flash2}},//f1
 {0x24,0x00,30,{flash3,flash3,flash3,flash3,flash3,flash3,flash3,flash3}},//f2
 {0x24,0x02,30,{flash4,flash4,flash4,flash4,flash4,flash4,flash4,flash4}},//f3
 {0x24,0x00,30,{flash5,flash5,flash5,flash5,flash5,flash5,flash5,flash5}},//f4
 {0x24,0x02,30,{flash6,flash6,flash6,flash6,flash6,flash6,flash6,flash6}},//f5
 {0x24,0x00,30,{goS,goW,goS,goW,walk,goW,goS,goW}},//f6
  
};
unsigned long S;  // index to the current state 
unsigned long Input;

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	portBinit();
	portEinit();
	portFinit();
  SysTick_Init();
	PLLinit();
	EnableInterrupts();
  
	S = goW;
	
	while(1){
		
		GPIO_PORTF_DATA_R = FSM[S].w_out;// output to PF3 and PF1
		 
		GPIO_PORTB_DATA_R = FSM[S].t_out;// output to PB0-5
		
    SysTick_Wait10ms(FSM[S].Time);// SysTick delay
    
		Input = GPIO_PORTE_DATA_R&0x07; //get input
    
		S = FSM[S].Next[Input]; // next state

  }
}



void portBinit(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x02;          // activate Port B
  delay = SYSCTL_RCGC2_R;          // allow time for clock to stabilize
                                   // no need to unlock
  GPIO_PORTB_AMSEL_R &= ~0x3F;     // disable analog functionality on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF;// configure PB5-0 as GPIO
  GPIO_PORTB_DIR_R |= 0x3F;        // make PB5-0 out
  GPIO_PORTB_AFSEL_R &= ~0x3F;     // disable alt funct on PB5-0
  GPIO_PORTB_DR8R_R |= 0x3F;       // enable 8 mA drive on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;        // enable digital I/O on PB5-0
}

void portEinit(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x10;          // activate Port E
  delay = SYSCTL_RCGC2_R;          // allow time for clock to stabilize
                                   // no need to unlock
	GPIO_PORTE_AMSEL_R &= ~0x07;     // disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x00000FFF;// configure PE2-0 as GPIO
  GPIO_PORTE_DIR_R &= ~0x07;       // make PE2-0 in
  GPIO_PORTE_AFSEL_R &= ~0x07;     // disable alt funct on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;        // enable digital I/O on PE2-0
}


void portFinit(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0F4-PF0        
}
