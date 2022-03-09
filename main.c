/*
 * File:   main.c
 * Author: Wx
 *
 * Created on March 9, 2022, 11:24 AM
 */


#include <xc.h> 
#include <avr/io.h>
#include <util/delay.h> 
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#ifndef F_CPU
    #define F_CPU 16000000UL
#endif


/*
 * Calculation used:
 * 
 * << System Calculations >> 
 * 
 * Required Levels: 180 Level <Reslution = log2(180) = 8-bits>
 * Required Frequency: 50Hz <T_x = 1/50 = 20mS>
 * 
 * T_Signal = T_x/Levels = 20mS / 100 = 200uS
 * 
 * << Controller Calculations >>
 * s
 * TMR1 Pre Loading value:
 * T_Signal = ( 4 * Pre_Scalar_Val * (65535 - TCNT1) )/( F_clk )
 * 
 * TCNT1 = 64735 <800 tick: TO MAX[0xFFFF]>
 *  
 */
void TMR1_INIT(void);
uint16_t Angle2Val(uint8_t);


volatile uint16_t TMR1_C          = 0;

volatile uint16_t PWM_DC[10]      = {0};

volatile uint16_t PRE_LOADING_VAL = 65500;

#define LEVELS ( (volatile const uint16_t) 2650 )

#define MIN_VAL ((const uint8_t)  79)
#define MAX_VAL ((const uint16_t) 160)

int main(void) 
{
   
    DDRB |= 0xFF;
    DDRD |= 0x73;
    
    TMR1_INIT();
    
    for(uint8_t i = 0; (i < 10); i++)
    {
        PWM_DC[i] = Angle2Val(90);
    }
        
    sei(); // Enable global interrupts
    
    while (1)
    {
        
    }
    
    return 0;
}

ISR(TIMER1_OVF_vect)
{
    TMR1_C += 1;
    
    // Make Pin low every (PWM_DC) S
    
    // BJ_DOF1_SM
    if(TMR1_C >= PWM_DC[0])
    {
        PORTB &= ~(1 << PB0); 
    }
    
    // BJ_DOF2_SM
    if(TMR1_C >= PWM_DC[1])
    {
        PORTB &= ~(1 << PB1); 
    }
     
    // MJ_DOF_SM
    if(TMR1_C >= PWM_DC[2])
    {
        PORTB &= ~(1 << PB2); 
    }
    
    // SJ_DOF1_SM
    if(TMR1_C >= PWM_DC[3])
    {
        PORTB &= ~(1 << PB3); 
    }
    
    // SJ_DOF2_SM
    if(TMR1_C >= PWM_DC[4])
    {
        PORTB &= ~(1 << PB4); 
    }
    
    // Pinky
    if(TMR1_C >= PWM_DC[5])
    {
        PORTD &= ~(1 << PD0); 
    }
    
    // Ring
    if(TMR1_C >= PWM_DC[6])
    {
        PORTD &= ~(1 << PD1); 
    }
    
    // Middle
    if(TMR1_C >= PWM_DC[7])
    {
        PORTD &= ~(1 << PD5); 
    }
       
    // Index
    if(TMR1_C >= PWM_DC[8])
    {
        PORTD &= ~(1 << PD6); 
    }
    
    // Thumb
    if(TMR1_C >= PWM_DC[9])
    {
        PORTD &= ~(1 << PD4); 
    }
    
    // Make the pin high every 20mS
    if(TMR1_C >= LEVELS)
    {
        PORTB |= 0xFF;
        PORTD |= 0xFF;
        
        TMR1_C = 0;
    }
    
    //TIFR &= ~(1 << TOV1);
    
    TCNT1 = PRE_LOADING_VAL;
    
    return;
}


void TMR1_INIT(void)
{
    
    TCNT1   = PRE_LOADING_VAL; // Will tick 800 tick 
    
    TCCR1B |= (0 << WGM12) | (1 << CS10); // Pre scalar <1:1>
    
    TIMSK  |=  (1 << TOIE1); // Enable Timer1 Overflow Interrupt.
    
    TIFR   &= ~(1 << TOV1);   // Clear Timer1 Overflow Interrupt Flag.
    
}//end TMR1_INIT.

uint16_t Angle2Val(uint8_t Angle)
{ 
    if(Angle >= 180)
        Angle = 180;
    
    else if(Angle <= 0)
        Angle = 0;
      
    float Val = (0.45 * Angle) + 79.0;
    
    return (Val+1);
}


