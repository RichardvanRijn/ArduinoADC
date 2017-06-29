#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

#define SET_BIT(var,pos) ((var) |= (1<<(pos)))
#define CLEAR_BIT(var,pos) ((var) &= ~(1<<(pos)))
#define TOGGLE_BIT(var,pos) ((var) ^= (1<<(pos)))
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define ReceiveSize 16
unsigned char index = 0;

char data[ReceiveSize];
bool handleBit = false;

bool ADCsize10 = true;
uint16_t adcReading10b = 0;
unsigned char adcReading8b = 0;
uint8_t adc8bit = 0;
bool adcdone = false;

bool FS = false;  //ADC full speed

ISR(USART_RX_vect)
{
    char U;
    U = UDR0;
    if (UCSR0A & (1<<FE0 | 1<<DOR0 | 1<<UPE0)) {
        volatile char X;
        X = U;
    }
    if (index >= ReceiveSize) {
        handleBit = true;
    }
    else if (U == '\n'){
        handleBit = true;
    }
    else {
        data[index] = U; 
        index++;
    }
}

ISR(ADC_vect)
    {
  if(ADCsize10 == true) {
    unsigned char low, high;
    low = ADCL;
    high = ADCH;
  
    adcReading10b = (high << 8) | low;
    adcdone = true;
  }
  else {
    adcReading8b = ADCH;
    adcdone = true;
  }
}

ISR(TIMER1_COMPA_vect)
{
    SET_BIT(ADCSRA,ADSC);

    TCNT1H = 0;
    TCNT1L = 0; 
}

void Transmit(char bufferT[])
{
    volatile static uint8_t i;
    for (i = 0; bufferT[i] != '\0'; i++) {
        while (~UCSR0A & 1 << UDRE0);
            UDR0 = bufferT[i];
    }
}

void handle(void){
    switch(data[0])
    {
    case 'S': //start
        if (FS) {
            SET_BIT(TIMSK1,OCIE1A);
        }
        else {
            SET_BIT(ADCSRA,ADATE);
            SET_BIT(ADCSRA,ADSC);
        }
    break;

    case 'R': //stop
        CLEAR_BIT(ADCSRA,ADATE);
        CLEAR_BIT(TIMSK1,OCIE1A);
    break;

    case 'M': //ADC MUX
        unsigned char temp;
        temp = (data[1] - '0');
        if (temp > 8) {
            break;
        }
        else {
            CLEAR_BIT(ADCSRA,ADATE);  //stop ADC
            ADMUX &= ~15;       //clear MUX
            ADMUX += temp;        //set MUX.
        }
    break;

    case 'B': //serial baud
        char bufferB[16];
        long baudRate;
        memcpy (bufferB,(data+1),strlen(data)); //copy buffer to prepare for sscanf
        sscanf(bufferB, "%ld", &baudRate);      //convert char[] to long

        uint16_t baudReg;
        baudReg = (((16000000UL)/(16*baudRate))-1);   //  ((UL16000000)/(16*BAUD))-1
        UBRR0 = baudReg;   
    break;

    case 'T': //ADC speed 0 = fullspeed
        char bufferT[16];
        uint16_t timerTime;
        memcpy (bufferT,(data+1),strlen(data)); //copy buffer to prepare for sscanf
        sscanf(bufferT, "%u", &timerTime);      //convert char[] to long
        if(timerTime != 0) {
            FS = false;
            TCNT1 = 0;
            OCR1A = timerTime;
            SET_BIT(TIMSK1,OCIE1A);
            CLEAR_BIT(ADCSRA,ADATE);
            SET_BIT(ADCSRA,ADSC);
        }
        else {
            FS = true;
            CLEAR_BIT(TIMSK1,OCIE1A);
            SET_BIT(ADCSRA,ADATE);
            SET_BIT(ADCSRA,ADSC);
        }   
    break;

    case 'D': //data size

     uint8_t ADCADLAR;
     memcpy (bufferT,(data+1),strlen(data)); //copy buffer to prepare for sscanf
     sscanf(bufferT, "%u", &ADCADLAR);      //convert char[] to long

    if(ADCADLAR > 1) {
      break;
    }
    if(ADCADLAR > 0) {
      SET_BIT(ADMUX,ADLAR);
      ADCsize10 = false;
    }
    else {
      CLEAR_BIT(ADMUX,ADLAR);
      ADCsize10 = true;

      char bufferADC[16];
        snprintf(bufferADC, sizeof bufferADC, "ADLAR1\n");
        Transmit(bufferADC);

        
    }
    break;

    default:

    break;
    }
    handleBit = false;
    index = 0;
}

void setup() {
    ADMUX = (1 << REFS0);
    ADCSRA |= (1 << ADIE);
    //  ADCSRA |= (1 << ADATE);

    UCSR0A = 0x00;
    //UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ00) /* | (1 << UPM01) */ | (1 << UCSZ01); //Even parity (no parity)
    UBRR0H = 0;
    UBRR0L = 51;
    //  BAUD = 19200 UBRRL = 51  F_CPU = 16000000 

    TCCR1A = 0;
    TCCR1B |= (1 << CS10);
    //  TCCR1B |= (1 << CS11) | (1 << CS10);
    //  TCCR1B |= (1 << CS12) | (1 << CS10);
    TCCR1C = 0;
    OCR1A = 32768;

    //TIMSK1 = (1 << OCIE1A);

    sei();
}

void loop() {
    if (handleBit){
        handle();
    }
    if (adcdone){
      cli();
      if(ADCsize10 == true) {
        char bufferADC[16];
        snprintf(bufferADC, sizeof bufferADC, "%i.\n",adcReading10b);
        Transmit(bufferADC);
        //    Transmit(adcReading);
      }
      else {
        while (~UCSR0A & 1 << UDRE0);
        UDR0 = adcReading8b;
       while (~UCSR0A & 1 << UDRE0);
         UDR0 = '.';
       while (~UCSR0A & 1 << UDRE0);
         UDR0 = '\n';
      }
     adcdone = false;
     sei();
    }
}

