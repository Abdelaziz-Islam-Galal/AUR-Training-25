#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint32_t time = 0;

ISR(TIMER0_OVF_vect)
{
  time++;
  TCNT0 = 256 - 250; // Reload timer for next overflow
}

void init_millis(void)
{
  TCCR0A = 0;
  TIMSK0 |= (1 << TOIE0); // overflow interrupt
  TCNT0 = 256 - 250; // 250 counts = 1ms at 16MHz/64
  TCCR0B |= (1 << CS01) | (1 << CS00); // prescaler 64
  
  sei();
}

uint32_t my_millis(void)
{
  uint32_t time_copy;
  cli();
  time_copy = time;
  sei();
  return time_copy;
}

int main(void)
{
  init_millis();
  DDRB |= (1 << 5); // led pin output
  
  uint32_t prev_time = 0;
  
  while (1)
  {
    uint32_t current_time = my_millis();
    
    // Use >= instead of == to handle missed comparisons
    if (current_time - prev_time >= 1000)
    {
      PORTB ^= (1 << 5);
      prev_time = current_time;
    }
  }
  
  return 0;
}