#define F_CPU 16000000UL

//#include <avr/io.h>

/*
//#include <util/delay.h>

// Основная функция программы.
void main(void) {
    //DDRC |= (1 << PC5);
    DDRB |= (1 << DDB5);
    while (1) { 
      PORTB |= (1 << DDB5);
      _delay_ms(500);
      PORTB &= ~(1 << DDB5);
      _delay_ms(500);
  }
}
*/

// Arduino таймер CTC прерывание
// avr-libc library includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void setup()
{
    //Led pin mode out
    DDRB |= (1 << DDB5);

    // инициализация Timer1
    cli();  // отключить глобальные прерывания
    TCCR1A = 0;   // установить регистры в 0
    TCCR1B = 0;

    OCR1A = 15624; // установка регистра совпадения

    TCCR1B |= (1 << WGM12);  // включить CTC режим 
    TCCR1B |= (1 << CS10); // Установить биты на коэффициент деления 1024
    TCCR1B |= (1 << CS12);

    TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 
    sei(); // включить глобальные прерывания
}

void main()
{
  setup();
    // основная программа
}

ISR(TIMER1_COMPA_vect)
{
  //PORTB ^= (1 << DDB5);
  PORTB |= (1 << DDB5);
  _delay_ms(100);
  PORTB &= ~(1 << DDB5);
  _delay_ms(100);
}
