#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint16_t TOP = 0xFFFF;

ISR(TIMER1_COMPA_vect)
{
  PORTB &= ~(1 << 5); // close
}
ISR(TIMER1_OVF_vect)
{
  PORTB |= (1 << 5); // open
  TCNT1 = 0xFFFF - TOP; // starting point for timer ticks
}

void PWM_init(uint16_t freq, uint8_t duty);
void PWM_setDuty(uint8_t duty);
void PWM_setFreq(uint16_t freq);

int main(void)
{
  PWM_init(1000, 0);

  while(1) {
    _delay_ms(100);
  }

  return 0;
}

void PWM_init(uint16_t freq, uint8_t duty)
{
  TCCR1A = 0; // timer 1 on normal mode
	TIMSK1 |= (1 << OCIE1A) | (1 << TOIE1); // overflow + compare (on register A) interrupts
	TCCR1B |= (1 << CS10); // no prescaler
  
	TCNT1 = 0 ; // reset the timer 

  PWM_setFreq(freq);
  PWM_setDuty(duty);
  
  DDRB |= (1 << 5); // led pin output
  PORTB &= ~(1 << 5);
  TCNT1 = 0xFFFF - TOP;
  
  sei();
}

void PWM_setDuty(uint8_t duty)
{
  // Duty_cycle = (cmp / TOP) * 100 -> in my case cmp is at OCR1A
  // hence: cmp = TOP * Duty_cycle / 100

  if(duty > 100) duty = 100; // 100% is max value

  uint16_t cmp = TOP * duty / 100;

  if(cmp >= TOP) cmp = TOP - 1;

  OCR1A = cmp;
}

void PWM_setFreq(uint16_t freq)
{
  // F_CPU = 16MHz, timer tick = 1/16MHz = 0.0625µs
  // Timer frequency = F_CPU / (N × (TOP + 1)) -> in my case: N=1 (prescaler)
  // TOP = (F_CPU / freq) - 1

  if(freq == 0) freq = 1;
  if(freq > 16000) freq = 16000;

  TOP = (16000000UL / freq) - 1;

  // better safe than sorry:
  if(TOP > 0xFFFF) TOP = 0xFFFF;
  if(TOP < 1) TOP = 1;

  TCNT1 = 0xFFFF - TOP;
}









