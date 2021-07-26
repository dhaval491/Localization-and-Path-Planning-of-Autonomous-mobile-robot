/*AVR CODE for FIREBIRD V by Hindustan Dynamics*/
/*
* Project Name:   <Vision Guided Autonomous Mobile Robot In Static Environment>
* Author List:  <Dhaval Patel, Jash Diyora> 
*
* Filename:   <Path Planning.m>
  
* Functions:  <USART_Init( unsigned int ubrr), motorconfig(void), encoder_config(void), format_counts()
    left(long int theta), right(long int theta), forward(long int d), int main(void)>
  
* Global Variables: <x, rightcount, leftcount, turncount, reqturncount, theta> 
*     variables>
*
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include<util/delay.h>
volatile unsigned long int rightcount=0;   // variable for right motor interruptcount
volatile unsigned long int leftcount=0;    // variable for left motor interrupt count
volatile unsigned long int reqturncount=0;   // variable for required interrupt count for angular positioning
float theta=0;      // variable for angular position
  /*
  * Function Name:  USART_Init
  * Input:    ubrr -> int for 115200 baudrate at clock frequency of 14745600hz according to atmega2560 datasheet
  * Output:   none
  * Logic:    Initialize Serial communication by setting proper USART registers
  * Example Call    USART_Init(7)
  *
  */
void USART_Init( unsigned int ubrr)
{
  UBRR3H = (unsigned char)(ubrr>>8);  // Set baud rate 
  UBRR3L = (unsigned char)ubrr;   
  UCSR3B = (1<<RXEN3)|(1<<TXEN3); //Enable receiver and transmitter 
  UCSR3C = (1<<USBS3)|(3<<UCSZ30);  // Set frame format: 8data, 2stop bit 
}
  /*
  * Function Name:  motorconfig
  * Input:    void
  * Output:   none
  * Logic:    Enables right and left motors
  * Example Call: motorcongif();
  *
  */
void motorconfig(void)
{
  DDRA=0xFF; //set motor pins to output
  DDRL=0xFF;//set motor enable pins to output
  PORTL=0b00011000;//right and left motor enable
}
  /*
  * Function Name: encoder_config
  * Input:    void
  * Output:   none
  * Logic:    Enables right and left motors encoders for interrupt counts
  * Example Call: encoder_congif();
  *
  */
void encoder_config(void)
{
  DDRE=0b11001111;//both motor config set
  PORTE=0x00;//set encoder pins as input for interrupt
  cli(); // clear all interrupts
  EICRB = 0x0A;// set the type of interruption
  EIMSK = 0x30; // mask interrupt ports for input
  sei(); // enables global interrupts
}
ISR(INT4_vect)//left encoder interrupt set to increase left encoder interrrupt count by 1 on every interrrupt
{
  leftcount++;
}
ISR(INT5_vect)//right encoder interrupt set to increase left encoder interrrupt count by 1 on every interrrupt
{
  rightcount++;
}
/*
  * Function Name: stop
  * Input:    void
  * Output:   none
  * Logic:    Stops motors by writing zero to motor register PORTA
  * Example Call: stop();
  *
  */
void stop()
{
  PORTA=0x00; // set PORTA to zero to stop motors
}
/*
  * Function Name: format_counts
  * Input:    void
  * Output:   none
  * Logic:    sets right and left motors encoder interrupt counts at zero
  * Example Call: format_countd();
  *
  */
void format_counts()// erase all counts
{
  rightcount=0; // set rightcount to zero for next motion
  leftcount=0;  // set leftcount to zero for next motion
}       
/*
  * Function Name: left
  * Input:    theta-> amount of angle in degrees by which robot has to take hard turn left
  * Output:   hard left turn of robot by angle of theta
  * Logic:    start motion of robot for hard left turn. Evaluates required interrupt count for a given
  *       rotation. When these counts are matched with interrupt counts of left motor
  *       motion of robot is stopped and all counts are set to zero.
  * Example Call: left(90);
  *
  */
void left(long int theta)
{
  int reqturncount=ceil(theta*(0.24052));   // calculating amount of right encoder interrupt counts that will lead
          // an agular displacement of theta.  
  PORTA=0x05;       // start motion in left direction
  while(1)
  {
    if(leftcount>reqturncount)  // when left motor interrupt count matches with required left interrupt 
          //count motion is set to stop and both interrupts count are stopped
    {
    stop();
    format_counts();
    break;
    }
  }
}
  /*
  * Function Name: right
  * Input:    theta-> amount of angle in degrees by which robot has to take hard turn right
  * Output:   hard right turn of robot by angle of theta
  * Logic:    start motion of robot for hard right turn. Evaluates required interrupt count for a given
  *       rotation. When these counts are matched with interrupt counts of right motor
  *       motion of robot is stopped and all counts are set to zero.
  * Example Call: right(60);
  *
  */
void right(long int theta)
{
  int reqturncount=ceil(theta*(0.24052));  // calculating amount of right encoder interrupt counts that will lead
          // an agular displacement of theta. 
  PORTA=0x0A;       // Set motors motion in hard right turn
  while(1)
  {
    if(rightcount>reqturncount) // when left motor interrupt count matches with required left interrupt 
          //count motion is set to stop and both interrupts count are stopped
    {
    stop();
    format_counts();
    break;
    }
  }
}
  /*
  * Function Name: forward
  * Input:    d-> amount of distance in centimeters by which robot has to move in forward direction
  * Output:   forward motion of robot by d centimeters
  * Logic:    start motion of robot in forward direction by setting proper registers. 
          valuates required interrupt count for a given
  *       rotation. When these counts are matched with interrupt counts of right motor
  *       motion of robot is stopped and all counts are set to zero.
  * Example Call: forward(100);
  *
  */
void forward(long int d)
{
int requiredcountright=d/0.544;         // computing total counts of encoder that are required for forward motion 
              //Encoder resolution is 0.544cm per count
while(1)
    {
        PORTA=0x06;         //set robot in forward motion
      if(rightcount>requiredcountright) // when right motor interrupt count matches with required interrupt count
                        // motion is stopped 
                        
        {
        stop();
        format_counts();
        break;  
        }
    }
}
  /*
  * Function Name: int main
  * Input:    none
  * Output:   motions of robot in forward, hard right and hard left direction
  * Logic:    initializes encoder for interrupt, enables left and right motors for motion, initializes USART for
  *       Serial communication via bluetooth module. Stores the values from UDR3 register in D. Check for the 
  *       Sequence for which the value is recieved and accordingly give motions of robot
  *       
  * Example Call: int main(void);
  *
  */

int main(void)
{
 unsigned long int x;         // declaring a int variable x for storing data of UBBR3 register that is recieved by 
          //bluetooth communication 
  motorconfig();      // enable motors
  encoder_config();       // enable encoders
  USART_Init(7);          // initialize serial communication
  stop();         // stop the bot if already moving
    while(1)          // loop to start motion according to data recieved on serial port
    {
      while(!(UCSR3A & 1<<RXC3));     // wait till data is recieved
      x = UDR3;       // store the data into variable x
    if(x==10)     // checks if data is recieved from MATLAB and not anyother device
      {
        while(!(UCSR3A & 1<<RXC3));     // wait till data is recieved
        x=UDR3;       // store the data into variable x
        if(x>0)
        {
          left(x);                    // data recieved on 2nd transefer then move robot in hard left direction
                // by amount x degrees
          stop();     // stop the robot after required position is achieved
        }
        while(!(UCSR3A & 1<<RXC3));      // wait till data is recieved
        x=UDR3;       // store the data into variable x
        if(x>0)
        {
          right(x);     // data recieved on 3nd transefer then move robot in hard right direction
                // by amount x degrees
          stop();     // stop the robot after required position is achieved
        }
      
        while(!(UCSR3A & 1<<RXC3));    // wait till data is recieved
        x=UDR3;       // store the data into variable x
        if(x>0)
        {
          forward(x);   // data recieved on 4nd transefer then move robot in forwad direction
                // by amount x centimenter
          stop();     // stop the robot after required position is achieved
        }
      }
    else
      {
        continue;         // If data is not recieved from MATLAB, do nothing and continue recieving data
      }
    } 

}


