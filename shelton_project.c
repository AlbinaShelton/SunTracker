/*Albina Shelton    */
/*SER 456           */
/*  Smart Sunflower */
#include <avr/io.h>
#include <USART.h>
#include <util/setbaud.h>
#include <util/delay.h>
  uint16_t voltagePC1;
  uint16_t voltagePC5 ;

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
     //setting 1/8 pre-scalar
    TCCR1B &= ~(1 << CS12);
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
    // setting voltage
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    // setting the clock prescalar to 8
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
    ADCSRA &= ~(1 << ADPS2);
    // auto trigger ADC
    ADCSRA |= (1 << ADATE);
    //  enabling the ADC
    ADCSRA |= (1 << ADEN);
    // free running mode
    ADCSRB &= ~(1 << ADTS0);
    ADCSRB &= ~(1 << ADTS1);
    ADCSRB &= ~(1 << ADTS2);



}

 uint16_t adc_read_PC1(void) {
     //ADC1 is enable
    ADMUX |= (1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX |= (1 << MUX2);
    ADMUX |= (1 << MUX3);
    //ADC start conversion
    ADCSRA |= (1 << ADSC);
    ADMUX |= (1 << ADLAR);
    voltagePC1 = ADC;
    return voltagePC1;
}

 uint16_t adc_read_PC5(void) {
     //ADC5 enable
    ADMUX |= (1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX |= (1 << MUX2);
    ADMUX &= ~(1 << MUX3);
    //ADC start conversion
    ADCSRA |= (1 << ADSC);
    ADMUX |= (1 << ADLAR);
    voltagePC5 = ADC;
    return voltagePC5;
}
int main(void) {
   
     //set pin for 
    DDRB |= 0b11000000;
    DDRC &= 0x00;
    //initialize PMW and USART
    initPWM();
    USART0_init();
     
    //voltage initialization
    float voltage5 = 0;
    float voltage5_temp;
    float voltageI = 0;
    float voltageI_temp;

    float pos=0;
    

    // Event Loop
    while (1) {

        //reading PC1 ADC value
        printString("\nLDR 1 : ");
        //init and read PC 1
        initADC();
        adc_read_PC1();
       //set temp value for LDR 1
        voltageI_temp = voltageI;
         // converting to PWM recognised values
        voltageI = (float) voltagePC1 / 256.00 * 5.00;
        //pritn value for LDR 1
        printWord(voltageI);
        _delay_ms(10);

        //Reading PC ADC value
        printString("\nLDR 5 : ");
        //init and read PC 5
        initADC();
        adc_read_PC5();
       //set temp value for  LDR 5
        voltage5_temp = voltage5;
        // converting to PWM recognised values
        voltage5 = (float) voltagePC5 / 256.00 * 5.00; 
        //print value from LDR 5
        printWord(voltage5);
        _delay_ms(10);

        //compare the value from LDR1 and LDR5
        if (voltageI <= (voltage5)) {
            //stay at same position
            if (voltage5 == voltage5_temp) {
                pos = pos;
            } else {
                //move back
                pos -= 30;
            }
        //compare the value from LDR5 and LDR1
        } else  {
            //stay at same position
            if (voltageI== voltageI_temp) {
                pos = pos;
            } else {
                //move front
                pos += 30;
            }

        }
        //set the min value for servo motor position 300
        if (pos < 300) {
            pos = 300;
        }
        //set the maximum value for servo motor position 2300
        if (pos > 2300) {
            pos = 2300;
        }
        //printing value for the servomotor position
        printString("\nServo motor position : ");
        printWord(pos);

        // set position  for OCR1B servo motor
        OCR1B = pos;
        _delay_ms(100);
    }
    return 0;
}

