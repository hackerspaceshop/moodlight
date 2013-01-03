
/// very simple interrupt-driven 8-bit PWM @ ~244 Hz
#define F_CPU	16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define CHMAX 3 // maximum number of PWM channels

#define RED_CLEAR (pinlevelB &= ~(1 << RED)) // map RED to PB0
#define GREEN_CLEAR (pinlevelB &= ~(1 << GREEN)) // map GREEN to PB1
#define BLUE_CLEAR (pinlevelB &= ~(1 << BLUE)) // map BLUE to PB2

//! Set bits corresponding to pin usage above
#define PORTB_MASK  (1 << PB0)|(1 << PB1)|(1 << PB2)

#define set(x) |= (1<<x)
#define clr(x) &=~(1<<x)
#define inv(x) ^=(1<<x)

#define RED PB0
#define GREEN PB1
#define BLUE PB2
#define LED_PORT PORTB
#define LED_DDR DDRB

void init(void);

unsigned char compare[CHMAX];
volatile unsigned char compbuff[CHMAX];

//exponential values
const char PW[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 27, 30, 33, 36, 39, 43, 47, 52, 56, 62, 67, 74, 81, 88, 96, 105, 115, 126, 137, 150, 164, 179, 196, 214, 234, 255};

int r_val = 0x00;
int g_val = 0x00;
int b_val = 0x00;

int main(void) {
  init();

  int r_dir = 1;
  int g_dir = 1;
  int b_dir = 1;

  int i;
  for(i=0;;i++) {
    if(!(i % 11)) r_val += r_dir;
    if(!(i % 13)) g_val += g_dir;
    if(!(i % 17)) b_val += b_dir;

    if(r_val == 0 || r_val == sizeof(PW)-1) r_dir *= -1;
    if(g_val == 0 || g_val == sizeof(PW)-1) g_dir *= -1;
    if(b_val == 0 || b_val == sizeof(PW)-1) b_dir *= -1;

    compbuff[0] = PW[r_val];
    compbuff[1] = PW[g_val];
    compbuff[2] = PW[b_val];

    _delay_ms(10);
  }
}


void init(void) {
  // set the direction of the ports
  LED_DDR set(RED);
  LED_DDR set(GREEN);
  LED_DDR set(BLUE);

  unsigned char i;

  CLKPR = (1 << CLKPCE);        // enable clock prescaler update
  CLKPR = 0;                    // set clock to maximum (= crystal)

  // initialise all channels
  for(i=0 ; i<CHMAX ; i++) {
    compare[i] = 0;             // set default PWM values
    compbuff[i] = 0;            // set default PWM values
  }

  TIFR = (1 << TOV0);           // clear interrupt flag
  TIMSK = (1 << TOIE0);         // enable overflow interrupt
  TCCR0B = (1 << CS00);         // start timer, no prescale

  sei();
}


ISR (TIM0_OVF_vect) {
  static unsigned char pinlevelB=PORTB_MASK;
  static unsigned char softcount=0xFF;

  PORTB = pinlevelB;            // update outputs

  if(++softcount == 0){         // increment modulo 256 counter and update
                                // the compare values only when counter = 0.
    compare[0] = compbuff[0];   // verbose code for speed
    compare[1] = compbuff[1];
    compare[2] = compbuff[2];

    pinlevelB = PORTB_MASK;     // set all port pins high
  }
  // clear port pin on compare match (executed on next interrupt)
  if(compare[0] == softcount) RED_CLEAR;
  if(compare[1] == softcount) GREEN_CLEAR;
  if(compare[2] == softcount) BLUE_CLEAR;
}
