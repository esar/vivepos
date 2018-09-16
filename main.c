#include <string.h>
#include <stdio.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"
#include "position.h"
#include "lighthouse_pos.h"

#define F_TIMER              ((F_CPU) / 4)
#define MICROSECONDS_IN_TICKS(x) ((uint16_t)((x) / 0.2))
#define NUM_SENSORS          4

#define PULSE_TYPE_SYNC      0x00
#define PULSE_TYPE_SWEEP     0x10
#define PULSE_TYPE_INVALID   0xf0
#define PULSE_HAS_DATA(x)    (((x) & 0xF0) == 0)
#define PULSE_DATA_AXIS(x)   ((x) & 1)
#define PULSE_DATA_DATA(x)   (((x) >> 1) & 1)
#define PULSE_DATA_SKIP(x)   (((x) >> 2) & 1)
#define PULSE_DATA_INVALID   0xFF

#define ANGLE_INVALID        0xFF

#define CHANGE_THRESHOLD     10
#define SENSOR_TIMEOUT       1000

#define SENSOR_STATE_IDLE    0
#define SENSOR_STATE_RISEN   1
#define SENSOR_STATE_FALLEN  2

#define NUM_LIGHTHOUSES      2
#define NUM_AXES             2
#define NUM_ANGLES           (NUM_LIGHTHOUSES * NUM_AXES)

#define OOTX_MAX_DATA        0x30

// TODO: Use RTC
// tested against a 76 ticks per second clock (20,000,000 / 4 / 65536)
#define REPORT_INTERVAL      15

typedef struct
{
	uint8_t  pos;
	uint16_t length;
	uint8_t  shiftCount;
	uint16_t shiftRegister;
	uint8_t  frameBit;
	uint8_t  validLength;
	uint8_t  data[OOTX_MAX_DATA];

} ootx_t;

typedef struct
{
	uint8_t isNew;
	uint8_t length;
	uint8_t data[OOTX_MAX_DATA];

} ootx_msg_t;

typedef struct
{
	volatile uint8_t  state;
	volatile uint16_t riseTime;
	volatile uint16_t fallTime;

	uint16_t angleTicks[NUM_ANGLES];
	_Accum angleRadians[NUM_ANGLES];
	uint16_t startTime;
	uint16_t firstPulseTime;
	uint8_t  currentAngle;

	ootx_t ootx[NUM_LIGHTHOUSES];

} sensor_t;

static ootx_msg_t g_lighthouseOotx[NUM_LIGHTHOUSES];
static sensor_t   g_sensors[NUM_SENSORS];
static uint8_t    g_time = 0;

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
		asm volatile( \
			       /* "sbi  0x15, 5         \n\t" */ \
			        "push r24             \n\t" \
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
			       /* "cbi  0x15, 5         \n\t" */ \
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

//VPORTF.OUT |= 64;
	// clear the flags so we know if another edge
	// arrives while we're working
	VPORTA.INTFLAGS |= triggered;

	while(triggered)
	{
		// Make sure we've got all the rising edges first
		if(TCB0.INTFLAGS & 1)
			TCB0_INT_vect();
		if(TCB1.INTFLAGS & 1)
			TCB1_INT_vect();
		if(TCB2.INTFLAGS & 1)
			TCB2_INT_vect();
		if(TCB3.INTFLAGS & 1)
			TCB3_INT_vect();

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
//VPORTF.OUT &= ~64;
}

// decodePulseWidth returns:
//     PULSE_TYPE_SWEEP (0xff)   for short sweep pulses
//     0x00 -> 0x07              for data from valid sync pulses
//     PULSE_TYPE_INVALID (0xf0) for pulses that are too long
//
// Pulse encoding info originally from: 
// https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md
//
//                      documented     observed     tick window with  
// skip | data | axis |  length uS | center ticks | (x - 309) / 51.2 
// -----+------+------+------------+--------------+-------------------
//    0 |    0 |    0 |       62.5 |    334 +/- 5 |       309 -> 359 
//    0 |    0 |    1 |       72.9 |    386 +/- 4 |       360 -> 410 
//    0 |    1 |    0 |       83.3 |    438 +/- 5 |       411 -> 461 
//    0 |    1 |    1 |       93.8 |    490 +/- 5 |       462 -> 512 
//    1 |    0 |    0 |      104.0 |    542 +/- 4 |       513 -> 563 
//    1 |    0 |    1 |      115.0 |    594 +/- 5 |       564 -> 615 
//    1 |    1 |    0 |      125.0 |    646 +/- 5 |       616 -> 666 
//    1 |    1 |    1 |      135.0 |    698 +/- 4 |       667 -> 717 
//
// Using 5Mhz clock, tick is 0.2uS
// 52 ticks between centers: (x * 5) / 256 = x / 51.2 = close enough
// width - 309 centers observed data in the length windows
//
uint8_t decodePulseWidth(int16_t width)
{
	uint8_t result;

	width = width - 296;
	if(width < 0)
		return PULSE_TYPE_SWEEP;
	result = ((width << 2) + width) >> 8;
	if(result > 7)
		result = PULSE_TYPE_INVALID;
	return result;
}

void ootxAddBit(ootx_t* ootx, uint8_t bit)
{
	if((bit & 1) && ootx->shiftRegister == 0 && ootx->frameBit == 0)
	{
		// got preamble
		ootx->pos = 0;
		ootx->length = 0;
		ootx->shiftCount = 0;
	}
	else if(ootx->shiftCount == 17)
	{
		ootx->shiftCount = 0;
		if((bit & 1) == 0)// || ootx->frameBit == 0)
			ootx->length = 0xff;  // abort

		// end of word
		if(ootx->length == 0)
		{
			ootx->length = (ootx->shiftRegister << 8) | (ootx->shiftRegister >> 8);
			ootx->length += 2;  // add checksum to length
		}
		else if(ootx->length != 0xff)
		{
			if(ootx->pos < ootx->length && ootx->pos < sizeof(ootx->data))
			{
				ootx->data[ootx->pos++] = ootx->shiftRegister >> 8;
				ootx->data[ootx->pos++] = ootx->shiftRegister & 0xFF;
			}
			else
			{
				// TODO: validate checksum
				ootx->validLength = ootx->length;
				ootx->length = 0xff;
			}
		}
	}

	ootx->frameBit = ootx->shiftRegister >> 15;
	ootx->shiftRegister = (ootx->shiftRegister << 1) | ((bit & 1));
	ootx->shiftCount = (ootx->shiftCount + 1);
}

uint8_t pollSensor(uint8_t sensorIndex)
{
	uint16_t now;
	sensor_t* sensor = &g_sensors[sensorIndex];

	if(sensor->state == SENSOR_STATE_RISEN)
	{
		now = TCA0.SINGLE.CNT;
		if(now - sensor->riseTime > SENSOR_TIMEOUT)
		{
			sensor->fallTime = sensor->riseTime + 1;
			sensor->state = SENSOR_STATE_FALLEN;
//printf("TOT\r\n");
VPORTF.OUT |= 16;
VPORTF.OUT &= ~16;
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

		//printf("%u, %u, %u, %u\r\n", sensorIndex, riseTime, width, data);

		if(data == PULSE_TYPE_INVALID)
		{
			printf("LRG %u\r\n", width);
			return 0;
		}

		if(data == PULSE_TYPE_SWEEP)
		{
			if(sensor->currentAngle < NUM_ANGLES)
			{
				uint16_t delta = riseTime - sensor->startTime;

				sensor->angleTicks[sensor->currentAngle] += delta;
 				sensor->angleTicks[sensor->currentAngle] >>= 1;
				sensor->angleRadians[sensor->currentAngle] = ((_Accum)delta - MICROSECONDS_IN_TICKS(4000)) * (M_PI / MICROSECONDS_IN_TICKS(8333));
				              
				//sensor->angleRadians[sensor->currentAngle] *= (M_PI / MICROSECONDS_IN_TICKS(8333));
				sensor->currentAngle = ANGLE_INVALID;
			}
		}
		else
		{
			uint16_t ticksSinceFirstPulse = riseTime - sensor->firstPulseTime;
			uint8_t lighthouse;

			if((ticksSinceFirstPulse >= MICROSECONDS_IN_TICKS(300) && 
			    ticksSinceFirstPulse <= MICROSECONDS_IN_TICKS(500)))
			{
				lighthouse = 1;
				sensor->firstPulseTime = riseTime - MICROSECONDS_IN_TICKS(400);
			}
			else
			{
				// Doesn't fit timing of second lighthouse, so it's either the first
				// or we're out of sync and might as well consider it the first
				lighthouse = 0;
				sensor->firstPulseTime = riseTime;
			}

			if(!PULSE_DATA_SKIP(data))
			{
				sensor->currentAngle = (lighthouse << 1) | PULSE_DATA_AXIS(data);
				sensor->startTime = riseTime;
			}
			ootxAddBit(&sensor->ootx[lighthouse], PULSE_DATA_DATA(data));
		}
	}
	else
	{
		now = TCA0.SINGLE.CNT;
		if(now - sensor->firstPulseTime > MICROSECONDS_IN_TICKS(8333 + 200))
		{
			// No pulse for more than 8333uS so move sensor->firstPulseTime
			// on by one cycle to try and stay locked while we're out of sight
			// of the lighthouses
			sensor->firstPulseTime += MICROSECONDS_IN_TICKS(8333);
		}
		else if(now - sensor->firstPulseTime > MICROSECONDS_IN_TICKS(600) &&
		        now - sensor->firstPulseTime < MICROSECONDS_IN_TICKS(800))
		{
			// Let the caller know that we're in the 1000uS processing
			// window at the end of the cycle where we don't expect any
			// pulses and can risk not being called for a while 
			return 1;
		}
	}

	return 0;
}

uint8_t processWindowCalcAndSendPos()
{
	static uint8_t sensorIndex = 0;
	vec3_t result;
	scalar_t distance;

VPORTF.OUT |= 2;
	calc_position(g_lighthouses,
	              g_sensors[sensorIndex].angleRadians[0],
	              g_sensors[sensorIndex].angleRadians[1],
	              g_sensors[sensorIndex].angleRadians[2],
	              g_sensors[sensorIndex].angleRadians[3],
	              result,
	              &distance);

	// 32 bytes per send
	printf("P:%u,%.1f,%.1f,%.1f,%.2f\r\n", 
	       sensorIndex,
	       (float)result[0], 
	       (float)result[1], 
	       (float)result[2], 
	       (float)distance);

	sensorIndex = (sensorIndex + 1) % 4;
VPORTF.OUT &= ~2;

	return 1;
}

// sends average of 29/4 = 7.25 bytes per sweep
uint8_t processWindowSendAngles()
{	
	static uint8_t count = 0;
	static uint16_t lastReportTime = 0;
	uint16_t now = g_time;
	uint8_t sensorIndex;

	// Only send updated angles once every 4 sweeps
	if(++count % 4 != 0)
		return 0;

	//if(now - lastReportTime > REPORT_INTERVAL*4)
	//{
		for(sensorIndex = 0; sensorIndex < NUM_SENSORS; ++sensorIndex)
		{
			// 29 bytes per send
			printf("A:%u,%u,%u,%u,%u\r\n", 
			       sensorIndex, 
			       g_sensors[sensorIndex].angleTicks[0], 
			       g_sensors[sensorIndex].angleTicks[1],
			       g_sensors[sensorIndex].angleTicks[2],
			       g_sensors[sensorIndex].angleTicks[3]);
		}
		lastReportTime = now;

		return 1;
	//}

	return 0;
}

uint8_t processWindowSaveOotx()
{
	uint8_t sensorIndex;
	uint8_t lighthouseIndex;
	uint8_t doneWork = 0;

	for(sensorIndex = 0; sensorIndex < NUM_SENSORS; ++sensorIndex)
	{
		for(lighthouseIndex = 0; lighthouseIndex < NUM_LIGHTHOUSES; ++lighthouseIndex)
		{
			ootx_t* ootx = &g_sensors[sensorIndex].ootx[lighthouseIndex];

			if(ootx->validLength)
			{
				g_lighthouseOotx[lighthouseIndex].length = ootx->validLength;
				memcpy(g_lighthouseOotx[lighthouseIndex].data, 
				       ootx->data, 
				       ootx->validLength);
				g_lighthouseOotx[lighthouseIndex].isNew = 1;
				ootx->validLength = 0;
				//printf("OOTX: %u/%u\r\n", sensorIndex, lighthouseIndex);
				doneWork = 1;
			}
		}
	}

	return doneWork;
}

uint8_t processWindowSendOotx()
{
	uint8_t lighthouseIndex;

	for(lighthouseIndex = 0; lighthouseIndex < NUM_LIGHTHOUSES; ++lighthouseIndex)
	{
		if(g_lighthouseOotx[lighthouseIndex].isNew)
		{
			uint8_t pos;

			// ~72 bytes per send
			printf("D:%u,%u,", 
			       lighthouseIndex, 
			       g_lighthouseOotx[lighthouseIndex].length);
			for(pos = 0; pos < g_lighthouseOotx[lighthouseIndex].length; ++pos)
				printf("%02x", g_lighthouseOotx[lighthouseIndex].data[pos]);
			printf("\r\n");
			g_lighthouseOotx[lighthouseIndex].isNew = 0;
			return 1;
		}
	}

	return 0;
}

int main()
{
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

	PORTF.OUT=0;
	PORTF.DIR=64|32|16|8 |4|2|1;
	PORTE.OUT=0;
	PORTE.DIR=8;

	PORTA.OUT = 1;      // Make USART0 pins outputs
        PORTA.DIR = 0xF;
	uartInit();

	sei();

	for(;;)
	{
		uint8_t inProcessingWindow;
		uint8_t sensorIndex;

VPORTF.OUT |= 8;
		inProcessingWindow = 0;
		for(sensorIndex = 0; sensorIndex < NUM_SENSORS; ++sensorIndex)
		{
			if(pollSensor(sensorIndex))
				inProcessingWindow = 1;
		}

		if(inProcessingWindow)
		{
			// per sweep we transmit:
			//    item     |  average           | worst case
			//    ---------+--------------------+-----------
			//    position | 32 bytes           | 32 bytes
			//    angles   | 29/4 = 7.25 bytes  | 29 bytes
			//    ootx     | 72/322 = 0.2 bytes | 72 bytes
			//    TOTAL    | 39.45 bytes        | 32+72 = 104 bytes
			//
			// average bytes/s: 39.45*240 = 9,468 bytes/s ~= 83Kbit/s
			// worst bytes/s: 104*240 = 24960 bytes/s ~= 220Kbit/s

			processWindowCalcAndSendPos();

			(void)(processWindowSendAngles() ||
			       processWindowSaveOotx() ||
			       processWindowSendOotx());
		}

VPORTF.OUT &= ~8;
	}

	return 0;
}
