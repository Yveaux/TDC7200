// Connections:
//
// TDC7200      Pro-Mini
//  1 Enable      4
//  2 Trigg       -
//  3 Start       3
//  4 Stop        5
//  5 Clock       8            8Mhz clock output configured in Pro Mini Fuse settings
//  6 Nc          -
//  7 Gnd         Gnd
//  8 Intb        2
//  9 Dout        12
// 10 Din         11
// 11 Csb         10
// 12 Sclk        13
// 13 Vreg        -           Via 1uF to Gnd
// 14 Vdd         Vcc 3.3V        

#include "TDC7200.h"

#define PIN_TDC7200_INT       (2)
#define PIN_TDC7200_ENABLE    (4)
#define PIN_TDC7200_STOP      (5)
#define PIN_TDC7200_START     (6)

#define PIN_TDC7200_SPI_CS    (10)
#define TDC7200_CLOCK_FREQ_HZ (F_CPU)

static TDC7200 tof(PIN_TDC7200_ENABLE, PIN_TDC7200_SPI_CS, TDC7200_CLOCK_FREQ_HZ);


#define NUM_STOPS (5)

void setup()
{
    Serial.begin(115200);
    Serial.println(F("-- Starting TDC7200 test --"));
    while (not tof.begin())
    {
        Serial.println(F("Failed to init TDC7200"));
        delay(1000);
    }

    pinMode(PIN_TDC7200_INT, INPUT_PULLUP);     // active low (open drain)

    digitalWrite(PIN_TDC7200_START, LOW);
    pinMode(PIN_TDC7200_START, OUTPUT);

    digitalWrite(PIN_TDC7200_STOP, LOW);
    pinMode(PIN_TDC7200_STOP, OUTPUT);

    if (not tof.setupMeasurement( 10,         // cal2Periods
                                  1,          // avgCycles
                                  NUM_STOPS,  // numStops
                                  2 ))        // mode
    {
        Serial.println(F("Failed to setup measurement"));
        while (1);
    }

    // Setup stop mask to suppress stops during the initial timing period.
//    tof.setupStopMask(121000000ull);

    // Setup overflow to timeout if tof takes longer than timeout.
//    tof.setupOverflow(130000000ull);
}

static void ui64toa(uint64_t v, char * buf, uint8_t base)
{
  int idx = 0;
  uint64_t w = 0;
  while (v > 0)
  {
    w = v / base;
    buf[idx++] = (v - w * base) + '0';
    v = w;
  }
  buf[idx] = 0;
  // reverse char array
  for (int i = 0, j = idx - 1; i < idx / 2; i++, j--)
  {
    char c = buf[i];
    buf[i] = buf[j];
    buf[j] = c;
  }
}

static void genPulse(const uint32_t usec, const uint8_t numStops)
{
    noInterrupts();
    digitalWrite(PIN_TDC7200_START, HIGH);
    digitalWrite(PIN_TDC7200_START, LOW);

    for (uint8_t i = 0; i < numStops; ++i)
    {
        delayMicroseconds(usec);

        digitalWrite(PIN_TDC7200_STOP, HIGH);
        digitalWrite(PIN_TDC7200_STOP, LOW);
    }
    interrupts();
}

void loop()
{
    static uint16_t pulseUs = 100;

    Serial.print(F("delay=")); Serial.print(pulseUs);

    tof.startMeasurement();

    genPulse(pulseUs, NUM_STOPS);

    // Wait for interrupt to indicate finished or overflow
    while (digitalRead(PIN_TDC7200_INT) == HIGH);

    for (uint8_t stop = 1; stop <= NUM_STOPS; ++stop)
    {
        uint64_t time;
        if (tof.readMeasurement(stop, time))
        {
            char buff[40];
            ui64toa(time, buff, 10);
            Serial.print(F("\ttof")); Serial.print(stop); Serial.print('='); Serial.print(buff);
        }
    }

    Serial.println();
    delay(1000);

/*
    pulseUs += 100;
    if (pulseUs > 2000)
    {
        pulseUs = 0;
    }

*/
}
