#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "nanomidi.h"

#define ROL32(x,b) (((x) << (b)) | ((x) >> (32 - (b))))

#define cbi(x,y) x&= ~_BV(y)
#define sbi(x,y)   x|= _BV(y)

#define FOSC 16000000 // Clock Speed
//#define BAUD 31250
#define BAUD 38400
#define MYUBRR (FOSC/16/BAUD-1)

#define CPU_MHZ (FOSC/1000000)

typedef struct midiNote_s{
	uint8_t cnt1,cnt2;
	uint8_t pres;
}midiNote_t;

midiNote_t midiNotes[120]={
	{55,139,2},{37,195,2},{227,240,1},{217,237,1},{37,164,2},{187,245,1},{184,235,1},{179,228,1},{172,224,1},{174,209,1},{143,240,1},{178,182,1},
	{139,220,1},{133,217,1},{120,227,1},{139,185,1},{148,164,1},{92,249,1},{92,235,1},{130,157,1},{86,224,1},{87,209,1},{131,131,1},{89,182,1},
	{110,139,1},{65,222,1},{60,227,1},{77,167,1},{74,164,1},{46,249,1},{46,235,1},{65,157,1},{43,224,1},{45,202,1},{39,220,1},{89,91,1},
	{55,139,1},{37,195,1},{227,240,0},{217,237,0},{37,164,1},{187,245,0},{184,235,0},{179,228,0},{172,224,0},{174,209,0},{143,240,0},{178,182,0},
	{139,220,0},{133,217,0},{120,227,0},{139,185,0},{148,164,0},{92,249,0},{92,235,0},{130,157,0},{86,224,0},{87,209,0},{131,131,0},{89,182,0},
	{110,139,0},{65,222,0},{60,227,0},{77,167,0},{74,164,0},{46,249,0},{46,235,0},{65,157,0},{43,224,0},{45,202,0},{39,220,0},{89,91,0},
	{55,139,0},{37,195,0},{30,227,0},{27,238,0},{37,164,0},{23,249,0},{23,235,0},{21,243,0},{28,172,0},{45,101,0},{22,195,0},{18,225,0},
	{21,182,0},{22,164,0},{15,227,0},{17,189,0},{37,82,0},{16,179,0},{17,159,0},{11,232,0},{14,172,0},{16,142,0},{11,195,0},{9,225,0},
	{13,147,0},{11,164,0},{13,131,0},{8,201,0},{37,41,0},{8,179,0},{7,193,0},{11,116,0},{7,172,0},{8,142,0},{29,37,0},{4,253,0},
	{4,239,0},{11,82,0},{23,37,0},{4,201,0},{3,253,0},{4,179,0},{4,169,0},{11,58,0},{7,86,0},{4,142,0},{4,134,0},{2,253,0},
};

uint16_t prescalers[5]={1,8,64,256,1024};
uint8_t presIdTimer1[5]={1,2,3,4,5};
uint8_t presIdTimer2[5]={1,2,4,6,7};

struct midi_buffer midi; 

//*****************************************************************************
// utils
//*****************************************************************************

void int2hex(unsigned long long n,unsigned char w,char * str)
{
  unsigned char i,d;

  str+=w;
  *str=0;

  for(i=0;i<w;++i)
  {
  	d= ( n >> (i << 2) ) & 0x0f;
    *--str= (d<=9)?d+48:d+55;
  };
}

void int2bin(unsigned long long n,unsigned char w,char * str)
{
  unsigned char i,d;

  str+=w;
  *str=0;

  for(i=0;i<w;++i)
  {
  	d=(n>>i)&1;
    *--str= (d)?'1':'0';
  };
}

int32_t lerp(int32_t x,int32_t y,int32_t alpha,int32_t alpha_scale)
{
	return x+alpha*(y-x)/alpha_scale;
}

//*****************************************************************************
// avr hw stuff
//*****************************************************************************

/* Delay for the given number of microseconds.  Assumes a 8 or 16 MHz clock. */
// Warning: Max delay time is 16384us = 16.384ms!
static inline void udelay(uint16_t us)
{
    // calling avrlib's delay_us() function with low values (e.g. 1 or
    // 2 microseconds) gives delays longer than desired.
    //delay_us(us);

#if CPU_MHZ >= 16
    // for the 16 MHz clock on most Arduino boards

    // for a one-microsecond delay, simply return.  the overhead
    // of the function call yields a delay of approximately 1 1/8 us.
    if (--us == 0)
        return;

    // the following loop takes a quarter of a microsecond (4 cycles)
    // per iteration, so execute it four times for each microsecond of
    // delay requested.
    us <<= 2;

    // account for the time taken in the preceeding commands.
    us -= 2;
#else
    // for the 8 MHz internal clock on the ATmega168

    // for a one- or two-microsecond delay, simply return.  the overhead of
    // the function calls takes more than two microseconds.  can't just
    // subtract two, since us is unsigned; we'd overflow.
    if (--us == 0)
        return;
    if (--us == 0)
        return;

    // the following loop takes half of a microsecond (4 cycles)
    // per iteration, so execute it twice for each microsecond of
    // delay requested.
    us <<= 1;

    // partially compensate for the time taken by the preceeding commands.
    // we can't subtract any more than this or we'd overflow w/ small delays.
    us--;
#endif

//    ATOMIC_START(h);
    // busy wait
    asm volatile (
        "1: sbiw %0,1" "\n\t" // 2 cycles
        "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
    );
//    ATOMIC_END(h);
}

// we lose approximately 3us on mdelay header & footer + 1ms on each udelay call
static inline void mdelay(uint16_t ms) {
    // max udelay = 16768us
    while (ms > 16) {
        ms -= 16;
        udelay(15999); // 15999us = 16ms - 1us (which is lost on call)
    }
    udelay(ms * 1000 - 3); // regain the lost 3us
}

void uart_init(void)
{
    // baud rate
    UBRRH=(unsigned char)(MYUBRR>>8);
    UBRRL=(unsigned char)MYUBRR;
    // enable tx & rx
    sbi(UCSRB,TXEN);
    sbi(UCSRB,RXEN);
	
	UCSRB |= (1 << RXCIE); // Enable the USART Recieve Complete interrupt (USART_RXC) 
}

void uart_putch(const unsigned char ch){
    // wait prev tx
    while(!(UCSRA&_BV(UDRE)));
    // write
    UDR=ch;
}

unsigned char uart_getch(void){
    // wait rx
    while(!(UCSRA&_BV(RXC)));
    return UDR;
}

unsigned char uart_peekch(void){
    if(!(UCSRA&_BV(RXC)))
        return 0;
    else
        return UDR;
}

void uart_puts(const char * s){
    char * p=(char *)s;
    while(*p) uart_putch(*p++);
}

unsigned long uart_getHexNumber(void){
    unsigned long res=0;
    unsigned char c;

    for(;;){
        c=uart_getch();

        if(c=='\r' || c=='\n'){
            return res;
        }else if (c>='0' && c<='9'){
            uart_putch(c);
            res=res<<4UL;
            res=res|(unsigned long)(c-'0');
        }else if (c>='a' && c<='f'){
            uart_putch(c);
            res=res<<4UL;
            res=res|(unsigned long)(c-'a'+0xa);
        }
    }
}

ISR(USART_RXC_vect) 
{ 
   unsigned char b; 
   b = UDR;

   midi_parse(&midi,&b,1);
}

//*****************************************************************************
// adc
//*****************************************************************************

#define ADCCHAN_DETUNE 0
#define ADCCHAN_MOD 1

void adc_init(void)
{
	// 2 ana inputs: PA6/PA7
	cbi(DDRA,PIN6);
	cbi(DDRA,PIN7);
	// VCC ref
    sbi(ADMUX,REFS0);
    cbi(ADMUX,REFS1);
    // 8 MSB
    sbi(ADMUX,ADLAR);
    // enable
    sbi(ADCSRA,ADEN);
	// freq=fosc/32
	sbi(ADCSRA,ADPS2);
	sbi(ADCSRA,ADPS0);
}

unsigned char adc_capture(const unsigned char channel)
{
    // sel channel
    ADMUX=(ADMUX & 0xe0) | (channel & 1) | 6;
    // mux is slow...
    udelay(10);
    // start capture
    sbi(ADCSRA,ADSC);
    // wait capture end
    while(ADCSRA&_BV(ADSC));
    // read
    return ADCH;
}

void eeprom_write(unsigned int uiAddress, unsigned char ucData)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
		;
	/* Set up address and data registers */
	EEAR = uiAddress;
	EEDR = ucData;
	/* Write logical one to EEMWE */
	EECR |= (1<<EEMWE);
	/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEWE);
}

unsigned char eeprom_read(unsigned int uiAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
		;
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}

//*****************************************************************************
// note osc
//*****************************************************************************

#define DACCLK PORTD2
#define DACDAT PORTD3
#define DACWRD PORTD4

#define OSC1OUT PORTB3
#define OSC2OUT PORTD5

#define NOTE_LOWEST 0
#define NOTE_HIGHEST 119
#define NOTE_NONE -1

int8_t osc2_detune=0;
uint16_t osc2_locnt=0,osc2_hicnt=0,osc2_curcnt=0;
int8_t osc1_octave=1;
int8_t osc2_octave_offset=0;

int duo_mode=0;
int8_t duo_chord_offset=0;

void note_init(void)
{
	cbi(DDRB,OSC1OUT);
	cbi(DDRD,OSC2OUT);

	DDRD|=0xDC; // DAC pins as output
	TCCR1A=0x40; // toggle OC1A
	TCCR1B=0x08; // CTC
	
	cbi(PORTD,DACWRD);
	
	TCCR0=0x1e;
	cbi(DDRB,PIN0);
	
	TCCR2=0x18;
}

void note_cvDacSend(int16_t value,int osc)
{
	int i;
	
	if(osc==1)
		cbi(PORTD,DACWRD);
	else
		sbi(PORTD,DACWRD);
	
	for (i=0;i<16;++i)
	{
		cbi(PORTD,DACCLK);
		
		if (value&0x8000)
			sbi(PORTD,DACDAT);
		else
			cbi(PORTD,DACDAT);

		udelay(10);

		sbi(PORTD,DACCLK);
		udelay(10);
		
		value<<=1;
	}
	
	if(osc==1)
		sbi(PORTD,DACWRD);
	else
		cbi(PORTD,DACWRD);

	for (i=0;i<5;++i)
	{
		cbi(PORTD,DACCLK);
		udelay(10);
		sbi(PORTD,DACCLK);
		udelay(10);
	}

	cbi(PORTD,DACCLK);
}

int16_t note_computeDetune(void)
{
	int16_t rv=osc2_curcnt;
	
	if (osc2_detune>0)
	{
		rv=lerp(osc2_curcnt,osc2_hicnt,osc2_detune,127);
	}
	else if (osc2_detune<0) 
	{
		rv=lerp(osc2_curcnt,osc2_locnt,-osc2_detune,127);
	}
	
	return rv;
}

void note_getDetune(void)
{
	osc2_detune=adc_capture(ADCCHAN_DETUNE)-128;
	OCR1A=note_computeDetune();
}

void note_play(int8_t note,int osc)
{
	if(note==NOTE_NONE)
	{
		if(osc==1)
			cbi(DDRB,OSC1OUT);
		else
			cbi(DDRD,OSC2OUT);
	}
	else
	{
		if(osc==1)
			sbi(DDRB,OSC1OUT);
		else
			sbi(DDRD,OSC2OUT);
	}
		
	if (note<NOTE_LOWEST || note>NOTE_HIGHEST) return;
	
	uint8_t cnt1=midiNotes[note].cnt1;
	uint8_t cnt2=midiNotes[note].cnt2;
	uint8_t pres=midiNotes[note].pres;
	
	uint32_t abscnt=(uint32_t)cnt1*cnt2*prescalers[pres];
	int32_t cv=33160710/abscnt;

	if(osc==1)
	{
		TCCR2=(TCCR2&~7)|presIdTimer2[pres];
		
		OCR2=cnt1-1;
		OCR0=cnt2-1;
	}
	else
	{
		TCCR1B=(TCCR1B&~7)|presIdTimer1[pres];

		uint16_t cnt=cnt1*cnt2-1;
		
		int lonote=note-1;					
		int hinote=note+1;					
		
		osc2_locnt=cnt;
		osc2_hicnt=cnt;
		osc2_curcnt=cnt;
		
		if (lonote>=NOTE_LOWEST){
			osc2_locnt=(midiNotes[lonote].cnt1*midiNotes[lonote].cnt2-1);
			// convert to current note prescaler
			osc2_locnt=(uint32_t)osc2_locnt*prescalers[midiNotes[lonote].pres]/prescalers[pres];
		}
		
		if (hinote<=NOTE_HIGHEST){
			osc2_hicnt=(midiNotes[hinote].cnt1*midiNotes[hinote].cnt2-1);
			// convert to current note prescaler
			osc2_hicnt=(uint32_t)osc2_hicnt*prescalers[midiNotes[hinote].pres]/prescalers[pres];
		}
				
		OCR1A=note_computeDetune();
	}

	if (cv<0) cv=0;
	if (cv>65535) cv=65535;

	static int16_t cvs[2];
	
	cvs[osc-1]=cv-32768;
	
	// need to send both CVs at once in that order for proper DAC operation
	note_cvDacSend(cvs[1],2);
	note_cvDacSend(cvs[0],1);
}

int note_otherOsc(int osc)
{
	return 3-osc;
}

void note_event(int8_t note, int pressed)
{
	static int prevnote[2]={NOTE_NONE,NOTE_NONE};
	static int nextosc=1;

	if (pressed)
	{
		// env retrigger
		if (PIND&_BV(PIN6))
		{
			cbi(PORTD,PIN6);
			udelay(1500);
		}
	
		// next occupied and other free -> change next
		if (prevnote[nextosc-1]!=NOTE_NONE && prevnote[note_otherOsc(nextosc)-1]==NOTE_NONE)
		{
			nextosc=note_otherOsc(nextosc);
		}
		
		sbi(PORTD,PIN6);
		
		if(duo_mode)
		{
			if (nextosc==1) note_play(note+12+osc1_octave*12,1); // osc1 is pitched 1 octave lower due to the series timers
			if (nextosc==2) note_play(note+osc1_octave*12,2);
		}
		else
		{
			note_play(note+12+osc1_octave*12,1); // osc1 is pitched 1 octave lower due to the series timers
			note_play(note+duo_chord_offset+(osc1_octave+osc2_octave_offset)*12,2);
		}

		prevnote[nextosc-1]=note;
		
		nextosc=note_otherOsc(nextosc);
	}

	if (!pressed)
	{
		if (duo_mode)
		{
			int i;
			for(i=1;i<=2;++i)
			{
				if (note==prevnote[i-1])
				{
					note_play(NOTE_NONE,i);
					prevnote[i-1]=-1;
				}
			}

			if (prevnote[0]==NOTE_NONE && prevnote[1]==NOTE_NONE)
			{
				cbi(PORTD,PIN6);
			}
		}
		else
		{
			if (note==prevnote[note_otherOsc(nextosc)-1])
			{
				cbi(PORTD,PIN6);
			}
		}
	}
}

//*****************************************************************************
// sequencer
//*****************************************************************************

#define SEQ_EEPROM_COUNT_ADDR 768
#define SEQ_EEPROM_NOTES_ADDR 769

#define SEQ_MAX_NOTES 255

#define SEQ_STATE_STOP 0
#define SEQ_STATE_PLAY 1
#define SEQ_STATE_REC 2

#define SEQ_EVENT_SILENCE -1
#define SEQ_EVENT_TIE -2
#define SEQ_EVENT_BACK -3

int8_t seq_state=SEQ_STATE_STOP;
uint8_t seq_step=0;
uint8_t seq_count=0;

void seq_init(void)
{
	sbi(PORTB,PORTB2); // INT2 pin as input
	sbi(GICR,INT2); // INT2 enable
}

void seq_updateCount(void)
{
	eeprom_write(SEQ_EEPROM_COUNT_ADDR,seq_step);
}

void seq_setState(int state)
{
	seq_step=0;

	if(state==SEQ_STATE_REC)
	{
		seq_updateCount();
	}
	
	if(state==SEQ_STATE_PLAY)
	{
		seq_count=eeprom_read(SEQ_EEPROM_COUNT_ADDR);
	}

	if(state==SEQ_STATE_STOP)
	{
		note_play(NOTE_NONE,1);
		note_play(NOTE_NONE,0);
	}

	seq_state=state;
}

void seq_event(int8_t event) // any positive event is a note
{
	if(seq_state!=SEQ_STATE_REC)
		return;
	
	if (event==SEQ_EVENT_BACK)
		seq_step=(seq_step)?seq_step-1:0;
	else
	{
		eeprom_write(SEQ_EEPROM_NOTES_ADDR+seq_step,event);
		++seq_step;
	}
	
	seq_updateCount();
}


ISR(INT2_vect) 
{ 
	int8_t event,prev=SEQ_EVENT_SILENCE; 
   
	if(seq_state!=SEQ_STATE_PLAY || !seq_count)
		return;

	event=eeprom_read(SEQ_EEPROM_NOTES_ADDR+seq_step);
	if (seq_step) prev=eeprom_read(SEQ_EEPROM_NOTES_ADDR+seq_step-1);
   
	if(event==SEQ_EVENT_SILENCE && prev>=0)
	{
		note_event(prev,0);
	}
	else if(event>=0)
	{
		note_event(event,1);
		if (prev>=0) note_event(prev,0);
	}

	seq_step=(seq_step+1)%seq_count;
}

//*****************************************************************************
// keyboard + buttons matrix
//*****************************************************************************

#define MATRIX_NUM 10
#define MATRIX_KB_FIRST 1
#define MATRIX_KB_LAST 0x28


uint8_t matrix_states[MATRIX_NUM]={0};

void matrix_init(void)
{
	DDRA|=0xf;
	DDRC=0;
}

int matrix_keystate(uint8_t key)
{
	return (matrix_states[key>>3]&(1<<(key&7)))!=0;
}

void matrix_event(uint8_t key, int pressed)
{
    char s[30];
	
	int2hex(key,2,s);
	uart_puts(s);
	if (pressed)
		uart_puts(" pressed\r\n");
	else
		uart_puts(" released\r\n");


	// System 5 wheel -> OSC1 octave
	if (pressed && key>=0x31 && key<=0x35)
	{
		osc1_octave=key-0x31;
	}
	
	// 1-5 chord keys -> OSC2 octage offset
	if (pressed && key>=0x43 && key<=0x47)
	{
		osc2_octave_offset=0x45-key;
		
		duo_mode=0;
		duo_chord_offset=0;
	}

	// 6 chord key -> duo mode
	if (pressed && key==0x42)
	{
		uint8_t i,n=0;
		
		for(i=MATRIX_KB_FIRST;i<=MATRIX_KB_LAST;++i)
			if (matrix_keystate(i))
				n++;
		
		if (n==2)
		{
			// chord mode
			int16_t v=0;
			
			for(i=MATRIX_KB_FIRST;i<=MATRIX_KB_LAST;++i)
				if (matrix_keystate(i))
				{
					v=i;
					break;
				}
			
			for(i=MATRIX_KB_LAST;i>=MATRIX_KB_FIRST;--i)
				if (matrix_keystate(i))
				{
					v=i-v;
					break;
				}
			
			duo_mode=0;
			duo_chord_offset=v;
		}
		else
		{
			// duo mode
			duo_mode=1;
			duo_chord_offset=0;
			
			note_play(NOTE_NONE,1);
			note_play(NOTE_NONE,2);
		}
	}
		
	// Keyboard
	if (key>=MATRIX_KB_FIRST && key<=MATRIX_KB_LAST)
	{
		note_event(8+key,pressed);
		if (pressed) seq_event(8+key);
	}
	
	// Record
	if (pressed && key==0x41)
	{
		if(seq_state==SEQ_STATE_REC)
			seq_event(SEQ_EVENT_BACK);
		else
			seq_setState(SEQ_STATE_REC);
	}

	// Key start/fill in
	if (pressed && key==0x39)
	{
		seq_event(SEQ_EVENT_SILENCE);
	}


	// Start/Stop
	if (pressed && key==0x40)
	{
		if(seq_state!=SEQ_STATE_PLAY)
			seq_setState(SEQ_STATE_PLAY);
		else
			seq_setState(SEQ_STATE_STOP);
	}
	
	// Intro/ending
	if (pressed && key==0x38)
	{
		seq_event(SEQ_EVENT_TIE);
	}
}

void matrix_scan(void)
{
	int i,j;
	
	for(i=0;i<MATRIX_NUM;++i)
	{
		PORTA=(PINA&0xf0)|i;

		udelay(100);
		
		uint8_t ps=PINC;
		
		uint8_t pa=ps^matrix_states[i];
		
		if (pa)
		{
			for(j=0;j<8;++j)
			{
				if (pa & (1<<j))
				{
					matrix_event(i*8+j,(ps&(1<<j))!=0);
				}
			}
		}
		
		matrix_states[i]=ps;
	}
}

//*****************************************************************************
// midi
//*****************************************************************************

void midi_event(struct midi_msg *msg)
{
  switch(msg->type) {
  case MIDI_NOTE_ON:
	note_event(msg->data[0],1);
    break;
  case MIDI_NOTE_OFF:
	note_event(msg->data[0],0);
    break;
  case MIDI_PROGRAM_CHANGE:
    break;
  }
}

//*****************************************************************************
// main
//*****************************************************************************

int __attribute__((noreturn)) main(void)
{
    uart_init();
	note_init();
	matrix_init();
	adc_init();
	seq_init();

    uart_puts("synth\r\n");
	
	midi_buffer_init(&midi); 
	midi.callback = midi_event;  // Set a callback
//	midi.channel_mask = 0xff;    // Listen to channels 0 to 7
  
	sei(); // Enable the Global Interrupt Enable flag so that interrupts can be processed 	

	for(;;){
		matrix_scan();
		note_getDetune();
	}
}
