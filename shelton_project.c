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

uint16_t adc_read_PC5() {
    ADMUX |= (1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX |= (1 << MUX2);
    ADMUX &= ~(1 << MUX3);

    ADCSRA |= (1 << ADSC);

    ADMUX |= (1 << ADLAR);

}

uint16_t adc_read_PC1() {
    ADMUX |= (1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX |= (1 << MUX2);
    ADMUX |= (1 << MUX3);

    ADCSRA |= (1 << ADSC);

    ADMUX |= (1 << ADLAR);

}

//static inline int calibrate(float voltage) {
//    int maxValue = 229.5;
//    int minValue = 2.55;
//    int flag = 0;
//    if ((voltage < maxValue) | (voltage > minValue)) {
//        flag = 1;
//    }
//    return flag;
//}

int main(void) {
    USART0_init();
    // for two green LED which signify states

    DDRB |= 0b11000000;
    DDRC &= 0x00;

    initPWM();

    // 8 bit since we are using 16 bit PWM
    uint8_t dutyCycle;
    //voltage 16 bit intialization
    uint16_t voltagePC1 = 0;
    uint16_t voltagePC5 = 0;
    float voltage5 = 0;
    float voltage5_temp;
    float voltageI = 0;
    float voltageI_temp;
    float diff = 0;
    float pos;
    //ADC start conversion

    // ------ Event loop ------ //
    while (1) {

        //reading PC1 ADC value
        printString("\nLoop : ");
        initADC();
        adc_read_PC1();
        voltagePC1 = ADC;
        voltageI_temp = voltageI;
        voltageI = (float) voltagePC1 / 256.00 * 5.00;
        printWord(voltageI);
        _delay_ms(100);

        //Reading PC ADC value
        printString("\nLoop2 : ");

        initADC();
        adc_read_PC5();
        voltagePC5 = ADC;
        voltage5_temp = voltage5;

        voltage5 = (float) voltagePC5 / 256.00 * 5.00; // converting to PWM recognised values

        printWord(voltage5);
        _delay_ms(100);

        if (voltageI >= (voltage5+2)) {
            if (voltageI == voltageI_temp) {
                pos = pos;
            } else {
                pos += 20;
            }

        } else if (voltage5 > voltageI) {
            if (voltage5 == voltage5_temp) {
                pos == pos;
            } else {
                pos = pos - 50;
            }

        }
        if (pos < 300) {
            pos = 300;
        }
        if (pos > 2300) {
            pos = 2300;
        }
        //pos=1700;
        printString("\nLoop3 : ");
        printWord(pos);
        //if voltage is maxvalue then assign the dutycycle 255
        //if (voltage >= 229.5) {
        //                dutyCycle = 2000; //255 = 100% of 255, hence dutyCycle is 100%
        //            }//if voltage is minvalue then assign the dutycycle 2055
        //            else if (voltage <= 2.55) {
        //                dutyCycle = 300;
        //            }//assign voltage
        //            else {
        //dutyCycle = voltageI * 100; //255 = 100% of 255, hence dutyCycle is 100%

        // }

        // set dutyCycle for OCR1B
        OCR1B = pos;
        _delay_ms(100);
    }
    return 0;
}

