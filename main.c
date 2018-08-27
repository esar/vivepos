#include <string.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define BAUD 115200
#include <util/setbaud.h>

#include "uart.h"

#define F_TIMER              ((F_CPU) / 4)
#define NUM_SENSORS          4

#define PULSE_TYPE_SYNC      0x00
#define PULSE_TYPE_SWEEP     0x10
#define PULSE_TYPE_INVALID   0xf0
#define PULSE_DATA_AXIS(x)   ((x) & 1)
#define PULSE_DATA_DATA(x)   (((x) & 2) >> 1)
#define PULSE_DATA_SKIP(x)   ((x) & 4)

#define AXIS_INVALID         0xFF

#define CHANGE_THRESHOLD     10
#define SENSOR_TIMEOUT       1000

#define SENSOR_STATE_IDLE    0
#define SENSOR_STATE_RISEN   1
#define SENSOR_STATE_FALLEN  2

// tested against a 76 ticks per second clock (20,000,000 / 4 / 65536)
#define REPORT_INTERVAL      15

typedef struct
{
	volatile uint8_t  state;
	volatile uint16_t riseTime;
	volatile uint16_t fallTime;

	uint16_t lastAngles[4];
	uint16_t angles[4];
	uint16_t startTime;
	uint8_t  currentAxis;
	uint8_t  changed;

} sensor_t;

static sensor_t g_sensors[NUM_SENSORS];
static uint8_t g_time = 0;

ISR(TCA0_OVF_vect)
{
	++g_time;
	TCA0.SINGLE.INTFLAGS |= 1;
}

// NOTE: TCBx.INTFLAGS is cleared by reading TCBx.CCMP
#define TCB_ISR(x)    \
	ISR(TCB##x##_INT_vect, ISR_NAKED) \
	{ \
		/* g_sensors[0].riseTime = TCB0.CCMP; */ \
		/* g_sensors[0].state = SENSOR_STATE_RISEN; */ \
		asm volatile("push r24             \n\t" \
		             "push r30             \n\t" \
		             "push r31             \n\t" \
		             "ldi  r30, lo8(%1)    \n\t"  /* Z = &g_sensors[x] */ \
		             "ldi  r31, hi8(%1)    \n\t" \
		             "lds  r24, %0         \n\t"  /* r24 = TCBx_CCMPL */ \
		             "std  Z+1, r24        \n\t" \
	       	             "lds  r24, %0+1       \n\t"  /* r24 = TCBx_CCMPH */ \
		             "std  Z+2, r24        \n\t" \
		             "ldi  r24, 1          \n\t"  /* r24 = SENSOR_STATE_RISEN */ \
		             "st   Z,   r24        \n\t" \
		             "pop  r31             \n\t" \
		             "pop  r30             \n\t" \
		             "pop  r24             \n\t" \
		             "reti                 \n\t" \
		             : : "i" (_SFR_IO_ADDR(TCB##x##_CCMP)), "i" (&g_sensors[x])); \
	}
TCB_ISR(0);
TCB_ISR(1);
TCB_ISR(2);
TCB_ISR(3);


ISR(PORTA_PORT_vect)
{
	// grab the timer count and flags ASAP
	uint16_t count = TCB0.CNT;
	uint8_t  triggered = VPORTA.INTFLAGS;

	// clear the flags so we know if another edge
	// arrives while we're working
	VPORTA.INTFLAGS |= triggered;

	while(triggered)
	{
		if(triggered & (1 << (4+0)))
		{
			if(g_sensors[0].state == SENSOR_STATE_RISEN)
			{
				g_sensors[0].fallTime = count;
				g_sensors[0].state = SENSOR_STATE_FALLEN;
			}
		}
		if(triggered & (1 << (4+1)))
		{
			if(g_sensors[1].state == SENSOR_STATE_RISEN)
			{
				g_sensors[1].fallTime = count;
				g_sensors[1].state = SENSOR_STATE_FALLEN;
			}
		}
		if(triggered & (1 << (4+2)))
		{
			if(g_sensors[2].state == SENSOR_STATE_RISEN)
			{
				g_sensors[2].fallTime = count;
				g_sensors[2].state = SENSOR_STATE_FALLEN;
			}
		}
		if(triggered & (1 << (4+3)))
		{
			if(g_sensors[3].state == SENSOR_STATE_RISEN)
			{
				g_sensors[3].fallTime = count;
				g_sensors[3].state = SENSOR_STATE_FALLEN;
			}
		}

		count = TCB0.CNT;
		triggered = VPORTA.INTFLAGS;
		VPORTA.INTFLAGS |= triggered;
	}
}

uint8_t decodePulseWidth(uint16_t width)
{
	if(width < 100)
		return PULSE_TYPE_SWEEP;
	else if(width < 356)
		return 0;
	else if(width < 408)
		return 1;
	else if(width < 460)
		return 2;
	else if(width < 512)
		return 3;
	else if(width < 564)
		return 4;
	else if(width < 616)
		return 5;
	else if(width < 668)
		return 6;
	else if(width < 720)
		return 7;
	else
		return PULSE_TYPE_INVALID;
}

void processSensor(uint8_t sensorIndex)
{
	sensor_t* sensor = &g_sensors[sensorIndex];

	if(sensor->state == SENSOR_STATE_RISEN)
	{
		uint16_t now = TCA0.SINGLE.CNT;
		if(now - sensor->riseTime > SENSOR_TIMEOUT)
		{
			sensor->fallTime = sensor->riseTime + 1;
			sensor->state = SENSOR_STATE_FALLEN;
printf("TOT\r\n");
		}
	}

	if(sensor->state == SENSOR_STATE_FALLEN)
	{
		uint16_t riseTime;
		uint16_t width;
		uint8_t  data;

		riseTime = sensor->riseTime;
		width = sensor->fallTime - sensor->riseTime;
		sensor->state = SENSOR_STATE_IDLE;

		data = decodePulseWidth(width);

		//printf("%u, %u, %u\r\n", riseTime, width, data);

		if(data == PULSE_TYPE_SWEEP && sensor->currentAxis != AXIS_INVALID)
		{
			sensor->angles[sensor->currentAxis] = riseTime - sensor->startTime;
			sensor->currentAxis = AXIS_INVALID;
		}
		else if(data == PULSE_TYPE_INVALID)
			//printf("too large: %u, %u, %u\r\n", riseTime, width, data);
			printf("LRG %u\r\n", width);
		else if(!PULSE_DATA_SKIP(data))
		{
			sensor->currentAxis = PULSE_DATA_AXIS(data);
			sensor->startTime = riseTime;
		}

/*
		if(sensor->angles[sensor->currentAxis] > sensor->lastAngles[sensor->currentAxis] + CHANGE_THRESHOLD || 
		   sensor->angles[sensor->currentAxis] < sensor->lastAngles[sensor->currentAxis] - CHANGE_THRESHOLD)
		{
			sensor->changed = 1;
			sensor->lastAngles[sensor->currentAxis] = sensor->angles[sensor->currentAxis];
		}

		if(sensor->changed)
		{
			printf("%u: X: %u, Y: %u\r\n", sensorIndex, sensor->angles[0], sensor->angles[1]);
			sensor->changed = 0;
		}
*/
	}
}

int main()
{
	uint16_t lastReportTime = 0;

	memset(g_sensors, 0, sizeof(g_sensors));

	CCP = 0xd8;             // enable protected register access
	CLKCTRL.MCLKCTRLB = 0;  // no clock prescaler, run at full 20MHz

	EVSYS.CHANNEL1 = EVSYS_GENERATOR_PORT1_PIN0_gc;  // PORTB0 -> CHANNEL1
	EVSYS.USERTCB0 = 2;                              // CHANNEL1 -> TCB0

	EVSYS.CHANNEL2 = EVSYS_GENERATOR_PORT0_PIN0_gc;  // PORTC0 -> CHANNEL2
	EVSYS.USERTCB1 = 3;                              // CHANNEL2 -> TCB1
	
	EVSYS.CHANNEL3 = EVSYS_GENERATOR_PORT1_PIN0_gc;  // PORTD0 -> CHANNEL3
	EVSYS.USERTCB2 = 4;                              // CHANNEL3 -> TCB2

	EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT0_PIN0_gc;  // PORTE0 -> CHANNEL4
	EVSYS.USERTCB3 = 5;                              // CHANNEL4 -> TCB3

	TCA0.SINGLE.CTRLA = 5;    // enable, CLK/4
	TCA0.SINGLE.INTCTRL = 1;  // enable overflow interrupt

	TCB0.CTRLB   = 2;  // input capture on event
	TCB0.EVCTRL  = 1;  // enable event, rising edge
	TCB0.INTCTRL = 1;  // enable capture interrupt
	TCB0.CTRLA   = 21; // enable, source TCA_CLK, synchronised to TCA

	TCB1.CTRLB   = 2;  // input capture on event
	TCB1.EVCTRL  = 1;  // enable event, rising edge
	TCB1.INTCTRL = 1;  // enable capture interrupt
	TCB1.CTRLA   = 21; // enable, source TCA_CLK, synchronised to TCA

	TCB2.CTRLB   = 2;  // input capture on event
	TCB2.EVCTRL  = 1;  // enable event, rising edge
	TCB2.INTCTRL = 1;  // enable capture interrupt
	TCB2.CTRLA   = 21; // enable, source TCA_CLK, synchronised to TCA

	TCB3.CTRLB   = 2;  // input capture on event
	TCB3.EVCTRL  = 1;  // enable event, rising edge
	TCB3.INTCTRL = 1;  // enable capture interrupt
	TCB3.CTRLA   = 21; // enable, source TCA_CLK, synchronsised to TCA

	TCA0.SINGLE.CTRLESET = 8; // Restart TCA and all synchronised TCBs to get same count in all

	PORTA.PIN4CTRL = 3; // interrupt on falling edge
	PORTA.PIN5CTRL = 3; // interrupt on falling edge
	PORTA.PIN6CTRL = 3; // interrupt on falling edge
	PORTA.PIN7CTRL = 3; // interrupt on falling edge

	PORTA.OUT = 1;      // Make USART0 pins outputs
        PORTA.DIR = 0xF;
	uartInit();

	sei();

	for(;;)
	{
		uint16_t now;
		int sensorIndex;

		for(sensorIndex = 0; sensorIndex < NUM_SENSORS; ++sensorIndex)
			processSensor(sensorIndex);

		now = g_time;
		if(now - lastReportTime > REPORT_INTERVAL)
		{
			for(sensorIndex = 0; sensorIndex < NUM_SENSORS; ++sensorIndex)
				printf("(%u, %u, %u)\r\n", sensorIndex, g_sensors[sensorIndex].angles[0], g_sensors[sensorIndex].angles[1]);
			lastReportTime = now;
		}
	}

	return 0;
}
