// Arduino таймер CTC прерывание
// avr-libc library includes
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
 
static int uart_putchar(char c, FILE *stream)
{
   if (c == '\n')
      uart_putchar('\r', stream);
   loop_until_bit_is_set(UCSR0A, UDRE0);
   UDR0 = c;
   return 0;
}

void USART_Init( unsigned int ubrr)
{
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  /* Enable receiver and transmitter */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  /* Set frame format: 8data, 1stop bit */
  UCSR0C = (3<<UCSZ00);
  /* Enable IRQ on rx byte*/
  UCSR0B |= (1 << RXCIE0);
}

void USART_Transmit( unsigned char data )
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) )
  ;

  /* Put data into buffer, sends the data */
  UDR0 = data;
}

void io_init()
{
  //Led pin mode out
  DDRB |= (1 << DDB5);
  
  //Turn  on/off botton
  PORTC &= ~(1 << DDC0);
  DDRC |= (1 << DDC0);

  //ADC input pin
  DDRC &= ~(1<<DDC1);
}

void adc_init()
{
  //Select ADC1 pin and AVcc reference voltage
  //ADMUX = (0<<REFS1)|(1<<REFS0)|(0 << MUX3)|(0 << MUX2)|(0 << MUX1)|(1 << MUX0);
  ADMUX = (0<<REFS1)|(1<<REFS0)|0x1;

  //Disable digital input buffer
  DIDR0 = 0x2;
  
  //Turn on ADC and set ADC clock prescaler to 128
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

void setup()
{
  io_init();
  USART_Init(MYUBRR);
  adc_init();
}

void main()
{
  uint16_t adc_data;
  int16_t  adc_data_no_mean_mean = 0;
  int16_t  adc_data_no_mean = 0;
  int16_t  adc_data_mean = 0;
  uint8_t  locked = 0;  

  setup();
  stdout = &mystdout;
  
  while (1) {
    ADCSRA |= (1<<ADSC);

    while((ADCSRA & (1 << ADSC))) {
    // just loop
    }
    adc_data = ADCL;
    adc_data += ADCH<<8;

    adc_data_no_mean = (int16_t)(adc_data) - (adc_data_mean>>6);

    adc_data_no_mean_mean += adc_data_no_mean - adc_data_no_mean_mean/16;

    if ( ( ( adc_data_no_mean_mean < 100 ) && ( adc_data_no_mean_mean > -100 ) ) && ( locked == 0 ) ) {
      locked = 1;
      printf("Locked\n");
    }

    if ( ((adc_data_no_mean > 15) || (adc_data_no_mean<-15)) && (( adc_data_no_mean_mean < 100 ) && ( adc_data_no_mean_mean > -100 ) ) ) {
      printf("Led on %d\n", adc_data_no_mean);
      PORTB ^= (1 << DDB5);
      _delay_ms(500);
    }

    //_delay_ms(100);
    //printf("ADC:%d %d %d %d\n", (int16_t)adc_data, adc_data_mean/64, adc_data_no_mean,adc_data_no_mean_mean);
    
    adc_data_mean   += (((int16_t)adc_data) - adc_data_mean/64);
  }
}
