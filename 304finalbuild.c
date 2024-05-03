// Sashank Rao ID: 32356918

#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c.h"
#include "SSD1306.h"


#define TRIG PB1
#define ECHO PB0
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters).
#define BUTTONPIN PC3

#define REDLED PC0  // The pin the LED is connected to
#define GREENLED PC1  // The pin the LED is connected to

#include "util/delay.h"
#define DIG2 PB2    //enables DIG2. ATmega328P pin PB2/Arduino Uno pin digits[0]
#define DIG3 PB3    //enables DIG3. ATmega328P pin PB1/Arduino Uno pin 9
#define DIG4 PB4    //enables DIG4. ATmega328P pin PB0/Arduino Uno pin 10
#include <stdbool.h>

// defines variables
volatile unsigned short pulse_len;
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
double distanceFeet;
double myDistance;
int OLED_counter = 0;

double oldDistance;
double lowestDistance;
int buttonflag = 1;                 // LED status flag
int fakeDistance;

unsigned char digit1, digit2, digit3;

int myCounter;  //this is the variable that will be displayed
int closerthansix; //will also be displayed next to counter

bool sameCustomer = false;
bool justLeft6ftZone = true;



double runUltraSonic();
void disp3Digits(int distance);


void initialize() {
    //for UltraSonic Sensor
    DDRB |= (1<<TRIG); // Sets the trigPin as an OUTPUT
    DDRB &= ~ (1<<ECHO); // Sets the echoPin as an INPUT
  
    //for led rewrite
    DDRC |= (1<<GREENLED) | (1<<REDLED);// Declare the LED as an output
    PORTC |= (1<<GREENLED); // Turn the LED on
    PORTC &= ~ (1<<REDLED); // Turn the LED off

    //4digit7segment
    DDRD = 0xFF; //7segment pins
    DDRB |= (1 << DIG2) | (1 << DIG3) | (1 << DIG4); //digit enables

    //OLED rewrite
    OLED_Init();
    OLED_DisplayString("COUNTER OPEN");

    oldDistance = 0.00;
    lowestDistance = 15.99;

    //button
    DDRC &= ~ (1<<BUTTONPIN);
    PORTC |= (1<<BUTTONPIN);

    TCCR0A = 0; //timer mode - normal
    TCCR0B = 0x05; //1024 prescaler

}

void loop()
{
    // read the state of the pushbutton value:
    if ((PINC & (1<<BUTTONPIN)) == 0) { //turned on
        buttonflag = ~buttonflag;
    }

    if (buttonflag == 1) 
    {
        //ultrasonic
        myDistance = runUltraSonic();

        //LED
        if (myDistance < 6.00) 
        {
            PORTC |= (1<<REDLED); // Turn LED on
            PORTC &= ~(1<<GREENLED); // Turn the LED off

            if (oldDistance == 0.00) 
            {
                oldDistance = myDistance; //first time
            } else { // not the first time
                if ((myDistance - oldDistance) > 0) // they are leaving
                { 
                    if (lowestDistance > myDistance) 
                    {
                        lowestDistance = myDistance; //record breaking low distance from this same person
                        closerthansix += 1;
                    }

                    if (sameCustomer == 0) 
                    {
                        myCounter = myCounter + 1; //diff person
                        sameCustomer = 1; //until this person leaves the 6ft range, person will be registered as the same person
                    }
                }
                oldDistance = myDistance;
            }
            justLeft6ftZone = 1;
        }

        else if ((myDistance > 6.00) && (myDistance < 10.00)) {
        PORTC |= (1<<GREENLED); // Turn the LED on
        PORTC &= ~(1<<REDLED); // Turn the LED off
        sameCustomer = 0;


        } else {
        PORTC &= ~(1<<REDLED);
        sameCustomer = 0;

    
        lowestDistance = 15.99;
        justLeft6ftZone = 0;

        }
    }

    //4digit7segment
    
    fakeDistance = myDistance * 10;
    //char digit[3] = {fakeDistance/100%10,fakeDistance/10%10,fakeDistance%10};
    //char digit[3] = {2,3,4};
    disp3Digits(fakeDistance);
    
    if(OLED_counter == 10){
        //OLED
        OLED_Clear();
        OLED_SetCursor(0, 0);
        OLED_DisplayString("Distance: ");
        OLED_DisplayNumber(10, myDistance, 3);
        OLED_DisplayString("\nNumber of customers: ");
        OLED_DisplayNumber(10, myCounter, 3);
        OLED_DisplayString("\nNumber of customers \n within 6ft: ");
        OLED_DisplayNumber(10, closerthansix, 3);
        OLED_counter = 0;

    }
    OLED_counter += 1;
}



double runUltraSonic() {
    double range;
    unsigned int timeToRisingEdge;
    unsigned int timeToFallingEdge;
    unsigned int pulseWidth;
    
    PORTB &= ~(1 << TRIG); // 5 usec pre-TRIG
    _delay_us(5);
    PORTB |= (1 << TRIG); // 10 us pulse
    _delay_us(10); // to ultrasound
    PORTB &= ~(1 << TRIG); // TRIG pin
    
    TCNT0 = 0;
    while ((PINB & (1<<ECHO))==0); //wait till ECHO goes high
    timeToRisingEdge = TCNT0;
    while (!(PINB & (1<<ECHO))==0); //wait till ECHO1 goes low
    timeToFallingEdge = TCNT0;
    pulseWidth = timeToFallingEdge - timeToRisingEdge;
    range = pulseWidth * 1.098; //one way distance to target in cm 
    range = range / 2.54; //distance in inches
    range = range / 12; //distance in ft
  
    return range; //in feet

}


void disp3Digits(int distance) {
    unsigned char ledDigits[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67, 0x80};
    char digits[3] = {distance/100%10,distance/10%10,distance%10};
    for (unsigned char i = 0; i < 5; i++) { //this code will refresh at 5x30=150 mS
        // PORTD = ledDigits[d3]; //10's digit
        // PORTB = (1 << DIG3) | (1 << DIG4); //enable DIG2, disable DIG3 & DIG4, 0000 0011
        // _delay_ms(10);
        PORTD = ledDigits[digits[0]]; //1's digit w decimal pt
        PORTB = (1 << DIG3) | (1 << DIG4); //enable DIG3, disable DIG2,DIG4  0000 0101
        _delay_ms(10);
        PORTD = ledDigits[digits[1]] + 128; //0.1's digit
        PORTB = (1 << DIG2) | (1 << DIG4); //enable DIG4, disable DIG2,DIG3 0000 0110
        _delay_ms(10);
        PORTD = ledDigits[digits[2]]; //0.1's digit
        PORTB = (1 << DIG2) | (1 << DIG3); //enable DIG4, disable DIG2,DIG3 0000 0110
        _delay_ms(10);
        PORTB = (1 << DIG2) | (1 << DIG3) | (1 << DIG4);
    }
}

int main(void)
{
    initialize();

    while(1)
    {
        loop();
    }

    return 0;
}
