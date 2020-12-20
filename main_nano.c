// Arduino таймер CTC прерывание
// avr-libc library includes
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

volatile char time;
volatile char rx_uart_data_i;

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
}

void timer_init()
{
    time = 0;
    
    // инициализация Timer1
    TCCR1A = 0;   // установить регистры в 0
    TCCR1B = 0;

    OCR1A = 15624; // установка регистра совпадения

    TCCR1B |= (1 << WGM12);  // включить CTC режим 
    TCCR1B |= (1 << CS10); // Установить биты на коэффициент деления 1024
    TCCR1B |= (1 << CS12);

    TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 
}

void setup()
{
  rx_uart_data_i = 0;

  cli();  // отключить глобальные прерывания
  io_init();
  USART_Init(MYUBRR);
  timer_init();
  sei(); // включить глобальные прерывания
}

void main()
{
  setup();
  stdout = &mystdout;
  // основная программа
  while(1);
}

volatile unsigned char          rx_uart_data [9];

unsigned char read_co2_cmd [9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
char i;

ISR(TIMER1_COMPA_vect)
{
  if( (time % 5) == 0 ) {
    PORTB ^= (1 << DDB5);

    for(i=0; i<9;i=i+1) {
      USART_Transmit(read_co2_cmd[i]);
      rx_uart_data_i = 0;
    }
  }
  time++;
}

unsigned short co2;

ISR(USART_RX_vect)
{
  //Rx and tx the same byte
  //rx_uart_byte = UDR0;
  //UDR0 = rx_uart_byte;
  //PORTB ^= (1 << DDB5);
  rx_uart_data[ rx_uart_data_i ] = UDR0;
  rx_uart_data_i++;
  
  if ( rx_uart_data_i == 9 ) {
    co2 = (rx_uart_data[2]<<8)+rx_uart_data[3];
    printf("\nCO2: %d, T: %d\n",co2,rx_uart_data[4]-40);
  }
}
