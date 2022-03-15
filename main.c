/*
 * File:   main.c
 * Author: Wx
 *
 * Created on February 28, 2022, 3:53 AM
 * Project to control motors    
 */

/* Important notes*
 * 
 * SET PORT TRIS OUPUT  => 1
 * SET PORT TRIS INPUT  => 0
 */

#include <xc.h> 
#include <avr/io.h>
#include <util/delay.h> 
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#ifdef F_CPU

#undef F_CPU
    #define F_CPU 16000000UL

#endif

//=============================================================


//======================================================






/* <CODE CONTROL> */

#define TECHNIQUE_1 true
#define TECHNIQUE_2 false
#define TECHNIQUE_3 false

#define Auto_sweep  true
#define Step_sweep  false













/* <TECHNIQUE_1 STUFFS> */

#if (TECHNIQUE_1)

#define MAX_RANGE 505  // <Angle:+180>
#define MIN_RANGE 200 //  <Angle:   0>

#define TOP_VAL             ((const uint16_t) 4999)
#define SERVO_MAX_RANGE_TOP ((const uint16_t)  600)   //  <MAX_RANGE=+90d>
#define SERVO_MIN_RANGE_TOP ((const uint8_t )  130)  //   <MIN_RANGE=-90d>

volatile uint8_t Angle_Position = 0;

void PWM_Init(void);
void PWM_Duty(uint8_t);
void INT_INIT(void);
const uint16_t Angle2Val(const uint16_t);

#endif /* <TECHNIQUE_1 STUFFS> */


/* <TECHNIQUE_2 STUFFS> */

#if (TECHNIQUE_2)

volatile bool SAY_HI = false;
volatile bool INIT   = false;
volatile uint8_t Angle_Position = 0;

void INIT_STATE(bool);
void SAYING_HI(bool);
void servo_write_PB(const uint8_t, uint16_t);
void servo_write_PD(const uint8_t, uint16_t);
void T1_FPWM_INIT(void);

#endif /* <TECHNIQUE_2 STUFFS> */




/* <TECHNIQUE_3 STUFFS> */

#if (TECHNIQUE_3)

volatile uint16_t TMR1_C          = 0;

volatile uint16_t  PWM_DC[10]     = {0}; // Servos PWM Control Variable.

volatile uint16_t PRE_LOADING_VAL = 65500;

#define LEVELS ( (volatile const uint16_t) 2650 )

#define MIN_VAL ((const uint8_t)  79)   // <Angle: 0.0>
#define MAX_VAL ((const uint16_t) 160) //  <Angle: 180.0>

uint16_t Angle2Val(uint8_t);
void TMR1_INIT(void);

#endif /* <TECHNIQUE_3 STUFFS> */

/*
 * Technique_3 Algorithm: (Interrupt 100%):
 * 
 * 
 * Technique_2 Algorithm: (Interrupt 20% - Polling 80%):
 * 
 * 1- Interrupt routine: Triggered every 20mS <50Hz>
 * 2- Polling   routine: Every (20mS/2 pow 12) set PIN[0:N] to low.
 * 
 * Technique_1 Algorithm: (Interrupt 100%):
 * 
 * 1- Every 20mS <50Hz>: set the PIN[0:N] to HIGH.
 * 2- Every 3mS: T0 Interrupt flag will be raised.
 * 3- When T0_IF is on it will <++OCR1B> 
 * 4- When TCNT1 reaches OCR1B value: set the PIN[0:N] to LOW
 * 
 */

volatile uint8_t c = 0;

int main(void) 
{
    
    DDRB |= 0xFF;  // All output.
    DDRD |= 0x73; // All output.
    
      
#if (TECHNIQUE_1) // Enable Technique_1 functions.
    
    OCR1A = TOP_VAL;    // Control the Signal Frequency. 
    OCR1B = MIN_RANGE; // Control Duty cycle. <Ton/T>
    
    INT_INIT();
    PWM_Init();
    PWM_Duty(255);
        
#endif /*TECHNIQUE_1*/
    
#if (TECHNIQUE_2) // Enable Tecnique_2 functions.
    
    INT_INIT();      // Enable INT0, INT1 
    T1_FPWM_INIT(); // Enable TIMER1 Mode.
    
#endif /*TECHNIQUE_2*/
    
#if (TECHNIQUE_3)
    
    TMR1_INIT();
    
    for(uint8_t i = 0; (i < 10); i++)
        PWM_DC[i] = Angle2Val(20 + 2*i);
    
#endif /*TECHNIQUE_3*/
    
    sei(); // Enable interrupts.
    
	while(1)
	{
#if (TECHNIQUE_2)
        
        SAYING_HI(SAY_HI);
        INIT_STATE(INIT);
          
#endif /*TECHNIQUE_2*/
       
	}
    
    
}//end main.

//======================================================

/* StarT: Error Handling  */

#if   (Angle_Position > 180)
Angle_Position = 180;
#elif (Angle_Position < 0)
Angle_Position = 0;
#endif

// Error handling . (SA)
#if (TECHNIQUE_1)
    #if ( (Auto_sweep) && (Step_Sweep) )

    #warning "Error: (AUTO_SWEEP && STEP_SWEEP) == 1"
    #warning "AUTO_SWEEP: TRUE"
        #define Auto_sweep true // High Pririoty Activation
    #endif
#endif

// Error handling . T1T2T3
#if ( (TECHNIQUE_1) && (TECHNIQUE_2) )

    #warning "Error: (TECHNIQUE_1 && TECHNIQUE_2) == 1"
    #warning "TECHNIQUE_1: TRUE"
        #define TECHNIQUE_1 true // High Pririoty Activation

#elif ( (TECHNIQUE_2) && (TECHNIQUE_3) )

    #warning "Error: (TECHNIQUE_1 && TECHNIQUE_2) == 1"s
    #warning "TECHNIQUE_2: TRUE"
        #define TECHNIQUE_2 true // High Pririoty Activation

#elif ( (TECHNIQUE_1) && (TECHNIQUE_3) )

    #warning "Error: (TECHNIQUE_1 && TECHNIQUE_2) == 1"s
    #warning "TECHNIQUE_1: TRUE"
        #define TECHNIQUE_1 true // High Pririoty Activation

#endif

// Disable the sweeping for more performance.
#if ( (TECHNIQUE_2) || (TECHNIQUE_3) )

#ifdef Auto_sweep
    #undef Auto_sweep 
        #define Auto_sweep false
#endif

#ifdef Step_sweep
    #undef Step_sweep 
        #define Step_sweep false
#endif

#endif /* TECHNIQUE_2 */

/* End: Error Handling */



// Start the technique 1 implementation.
#if (TECHNIQUE_1)

// Technique_2 functions.
/*
 * Mode: CTC
 * Prescaler: 1:1
 */

void PWM_Init(void)
{
    TCCR1B |= (1 << CS10) | (1 << CS11) | (1 << WGM12);
    
    TIMSK  |= (1 << OCIE1A) | (1 << OCIE1B);
}

void PWM_Duty(uint8_t Time_val)
{
    if(Time_val > 255)
        Time_val = 255;
    
    else if(Time_val <= 1)
        Time_val = 1;
    else
        Time_val = 1;
    
    TCCR0 |= (1 << WGM01); // CTC MODE
    
    TCCR0 |= (1 << CS02) | (1 << CS00); // 1024 Pre scalar
    
    TIMSK |= (1 << OCIE0);
    
    OCR0 = (Time_val*7812.5) - 1;
}

void INT_INIT(void)
{
    PORTD |= (0 << PD2) | (0 << PD3);
    
    // Rising edge - exINT[0:1]
    MCUCR |= (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00);
    // Enable exINT[0:1]
    GICR  |= (1 << INT0) | (1 << INT1); 
}

const uint16_t Angle2Val(const uint16_t Angle)
{
    float A2V = Angle + 176.64;
          A2V /= 0.7059;
    
    return (A2V-2);
}


/* TECHNIQUE_1: INTERRUPTS */

ISR(TIMER1_COMPA_vect)
{
    PORTB |= 0xFF;
    PORTD |= 0xFF;
}


ISR(TIMER1_COMPB_vect)
{   
    PORTB &= ~PORTB;
    PORTD &= ~PORTD;      
}


ISR(TIMER0_COMP_vect)
{
    
#if (Auto_sweep)

    OCR1B += 1;
    
    if(OCR1B > MAX_RANGE)
        OCR1B = MIN_RANGE;  

    
#elif (Step_sweep)
    OCR1B = Angle2Val(Angle_Position);  
    
#endif
}

#if (Step_sweep)
ISR(INT0_vect)
{
    if(Angle_Position >= 180)
        Angle_Position = 180;
    else
        Angle_Position += 5;
}

ISR(INT1_vect)
{
    if(Angle_Position <= 0)
    Angle_Position = 0;
    else
        Angle_Position -= 5;
}
#endif // Step sweep.

#endif /*TECHNIQUE_1*/

// Start the technique 2 implementation.
#if (TECHNIQUE_2)


/* 
 * Fast PWM Mode <TOP:ICR1, BOT:0x00, TOV: TOP>
 * Pre scalar: /64
 * 
 * */

void T1_FPWM_INIT(void)
{
    TCCR1A |= (1 << WGM11);
    
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS10) | (1 << CS11);
    
    TIMSK  |= (1 << OCIE1A);
    
    ICR1 = 4999;
}

void INT_INIT(void)
{
    PORTD |= (0 << PD2) | (0 << PD3);
    
    // Rising edge - exINT[0:1]
    MCUCR |= (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00);
    // Enable exINT[0:1]
    GICR  |= (1 << INT0) | (1 << INT1); 
}

const uint16_t Angle2Val(const uint16_t Angle)
{
    float A2V = Angle + 176.64;
          A2V /= 0.7059;
    
    return (A2V-2);
}


void servo_write_PB(const uint8_t PIN, uint16_t Angle)
{

    if( (TCNT1 >= Angle) && (bit_is_set(PORTB, PIN)) )
    {
        PORTB &= ~(1 << PIN);
    } 
}

void servo_write_PD(const uint8_t PIN, uint16_t Angle)
{
    
    if( (TCNT1 >= Angle) && (bit_is_set(PORTD, PIN)) )
    {
        PORTD &= ~(1 << PIN);
    } 
}

void SAYING_HI(bool state)
{
    if(state)
    {
                /* - Shoulder */
        // * <BJ DOF1 - Rotation: PB0>
        servo_write_PB(PB0, Angle2Val(90));
        // * <BJ DOF2 - U/D     : PB1>
        servo_write_PB(PB1, Angle2Val(100));
        
        /* - Elbow */
        // * <MJ DOF1 - U/D     : PB2>
        servo_write_PB(PB2, Angle2Val(110));
        
        // * - Wrist
        // * <SJ DOF1 - Rotation: PB3>
        servo_write_PB(PB3, Angle2Val(120));
        // * <SJ DOF2 - U/D     : PB4>
        servo_write_PB(PB4, Angle2Val(130));
        
        /* - Hand */
        // * <Pinky: PD0> 
        servo_write_PD(PD0, Angle2Val(140));
        
        // * <Ring: PD1>
        servo_write_PD(PD1, Angle2Val(150));
        
        // * <Middle: PD5> 
        servo_write_PD(PD5, Angle2Val(160));
        
        // * <Index: PD6> 
        servo_write_PD(PD6, Angle2Val(170));
        
        // * <Thumb: PD4>
        servo_write_PD(PD4, Angle2Val(180));
    }
}

void INIT_STATE(bool state)
{
    if(state)
    {
                /* - Shoulder */
        // * <BJ DOF1 - Rotation: PB0>
        servo_write_PB(PB0, Angle2Val(0));
        // * <BJ DOF2 - U/D     : PB1>
        servo_write_PB(PB1, Angle2Val(0));
        
        /* - Elbow */
        // * <MJ DOF1 - U/D     : PB2>
        servo_write_PB(PB2, Angle2Val(0));
        
        // * - Wrist
        // * <SJ DOF1 - Rotation: PB3>
        servo_write_PB(PB3, Angle2Val(0));
        // * <SJ DOF2 - U/D     : PB4>
        servo_write_PB(PB4, Angle2Val(0));
        
        /* - Hand */
        // * <Pinky: PD0> 
        servo_write_PD(PD0, Angle2Val(0));
        
        // * <Ring: PD1>
        servo_write_PD(PD1, Angle2Val(0));
        
        // * <Middle: PD5> 
        servo_write_PD(PD5, Angle2Val(0));
        
        // * <Index: PD6> 
        servo_write_PD(PD6, Angle2Val(0));
        
        // * <Thumb: PD4>
        servo_write_PD(PD4, Angle2Val(0));
        
    }
}

/* TECHNIQUE_2: INTERRUPTS */

ISR(TIMER1_COMPA_vect)
{
    PORTB |= 0xFF;
    PORTD |= 0xFF;
}

ISR(INT0_vect)
{
    SAY_HI = true;
    
    INIT   = false;
}

ISR(INT1_vect)
{
    INIT   = true;
    
    SAY_HI = false;
}

#endif

/*
 * TECHNIQUE3 
 * 
 * TIMER1: NORMAL MODE.
 * PRE_SCALAR: 1:1
 * INTERRUPT: OVFLOW <START: TCNT1, END: 0xFFFF>
 */

#if (TECHNIQUE_3)


void TMR1_INIT(void)
{
    
    TCNT1 = PRE_LOADING_VAL;
    
    TCCR1B |= (0 << WGM12) | (1 << CS10); // Pre scalar <1:1>
    
    TIMSK |=  (1 << TOIE1); // Enable Timer1 Overflow Interrupt.
    
    TIFR  &= ~(1 << TOV1);   // Clear Timer1 Overflow Interrupt Flag.
    
}//end TMR1_INIT.

// Mapping Function.
uint16_t Angle2Val(uint8_t Angle)
{ 
    if(Angle >= 180)
        Angle = 180;
    
    else if(Angle <= 0)
        Angle = 0;
      
    float Val = (0.45 * Angle) + 79.0;
    
    return (Val+1);
}

/* TECHNIQUE_3: INTERRUPTS */

ISR(TIMER1_OVF_vect)
{
    TMR1_C += 1;
    
    // Make Pin low every (PWM_DC)
    
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
    
    // Make the pin high every 20mS with respect to the system.
    if(TMR1_C >= LEVELS)
    {
        PORTB |= 0xFF;
        PORTD |= 0xFF;
        
        TMR1_C = 0;
    }
    
    //TIFR &= ~(1 << TOV1); // Clear the TOV1 Flag bit.
    
    TCNT1 = PRE_LOADING_VAL; // Recharge the Timer1 Counter
    
    return;
}

#endif