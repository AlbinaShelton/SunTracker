/*Albina Shelton     */
/*SER 456            */
#include <avr/io.h>
#include <USART.h>
#include <avr/interrupt.h>    // Needed to use interrupts   
#include <util/setbaud.h>
#include <util/delay.h>
//volatile uint8_t flag;
//volatile uint16_t voltageI;
//volatile uint16_t voltageII;
//float voltageFirst;
//float voltageSecond;

void USART0_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
    //Enable USART transmitter / receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    //Set packet size (8 data bits)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    //Set stop bit amount 
    //UCSR0C &= ~(1 << USBS0);
    UCSR0C = (3 << UCSZ00);
}

static inline void initPWM(void) {
    // set WGM13:10 to 0b1010(13 and 11 in TCCR1A and otger in TCCR1B)
    TCCR1B &= ~(1 << WGM13);
    TCCR1B |= (1 << WGM12);
    TCCR1A &= ~(1 << WGM11);
    TCCR1A |= (1 << WGM10);
    // set CS12:10(all located in TCCR1B)
    TCCR1B &= ~(1 << CS12); //setting 1/8 pre-scalar
    TCCR1B &= ~(1 << CS10);
    TCCR1B |= (1 << CS11);
    // set COM1B0:1(all located in TCCR1A)
    TCCR1A &= ~(1 << COM1B0);
    TCCR1A |= (1 << COM1B1);
    // set PB2 as output(OC1B is PB2)
    DDRB |= (1 << PB2);
}

static inline void initADC(void) {
    // -------- Inits --------- //
    // 1. setting voltage
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    // 2.setting the clock prescalar to 8
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
    ADCSRA &= ~(1 << ADPS2);
    // 3.auto trigger ADC
    ADCSRA |= (1 << ADATE);
    // 4. enabling the ADC
    ADCSRA |= (1 << ADEN);
    // 5. free running mode
    ADCSRB &= ~(1 << ADTS0);
    ADCSRB &= ~(1 << ADTS1);
    ADCSRB &= ~(1 << ADTS2);
    //ADC5 enable 
    //if (count == 0) {
    ADMUX |= (1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX |= (1 << MUX2);
    ADMUX &= ~(1 << MUX3);
    ADCSRA |= (1 << ADSC);
    ///changing the allingment
    ADMUX |= (1 << ADLAR);
    //    }else if (count ==1){
    //        ADMUX=0;
    //    }
}

static inline int calibrate(float voltage) {
    int maxValue = 229.5;
    int minValue = 2.55;
    int flag = 0;
    if ((voltage < maxValue) | (voltage > minValue)) {
        flag = 1;
    }
    return flag;
}

int main(void) {
    USART0_init();
    // for two green LED which signify states
  
    DDRB |= 0b11000000;
    DDRC &= 0x00;
    // ----------Initialize------------//
    initPWM();
    initADC();
    // 8 bit since we are using 16 bit PWM
//    uint8_t dutyCycle = 0;
//    //voltage 16 bit intialization
//    uint16_t voltagePC0 = 0;
//    float voltage = 0;
    //ADC start conversion

    // ------ Event loop ------ //
    while (1) {
        printString("\nLoop : ");

        voltagePC0 = ADC;
        //conversion
        voltage = (float) voltagePC0 / 256.00 * 5.00; // converting to PWM recognised values
        //check if value is between calibrated range
        printWord(voltage);
        int flag = calibrate(voltage);
        if (flag == 1) {
            //if voltage is maxvalue then assign the dutycycle 255
            if (voltage >= 229.5) {
                dutyCycle = 2000; //255 = 100% of 255, hence dutyCycle is 100%
            }//if voltage is minvalue then assign the dutycycle 2055
            else if (voltage <= 2.55) {
                dutyCycle = 300;
            }//assign voltage
            else {
                dutyCycle = voltage * 100; //255 = 100% of 255, hence dutyCycle is 100%
            }
        }
        // set dutyCycle for OCR1B
        OCR1B = dutyCycle;
        _delay_ms(500);
    } /* End event loop */
    return 0;
}

//ISR(ADC_vect)
//{
//    uint8_t tmp;            // temp register for storage of misc data
//
//    tmp = ADMUX;            // read the value of ADMUX register
//    tmp &= 0x0F;            // AND the first 4 bits (value of ADC pin being used) 
//
//    ADCvalue = ADCH;        // read the sensor value
//
//    if (tmp == 0)
//    {
//        // put ADCvalue into whatever register you use for ADC0 sensor
//        ADMUX++;            // add 1 to ADMUX to go to the next sensor
//    }
//    
//    else if (tmp == 1)
//    {
//        // put ADCvalue into whatever register you use for ADC1 sensor
//         ADMUX &= 0xF8;  // clear the last 4 bits to reset the mux to ADC0
//    }
// 
//}