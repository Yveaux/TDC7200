#include "TDC7200.h"
#include <SPI.h>

#define TDC7200_SPI_CLK_MAX  (int32_t(20000000))
#define TDC7200_SPI_MODE     (SPI_MODE0)
#define TDC7200_SPI_ORDER    (MSBFIRST)
#define TDC7200_SPI_REG_ADDR_MASK (0x1Fu)
#define TDC7200_SPI_REG_READ      (0x00u)
#define TDC7200_SPI_REG_WRITE     (0x40u)
#define TDC7200_SPI_REG_AUTOINC   (0x80u)
#define TDC7200_ENABLE_LOW_MS            (5)
#define TDC7200_ENABLE_T3_LDO_SET3_MS    (2)

#define TDC7200_RING_OSC_FREQ_HZ               (144000000)
#define TDC7200_RING_OSC_FREQ_MHZ              (TDC7200_RING_OSC_FREQ_HZ/1000000)
#define PS_PER_SEC (1000000000000)
#define US_PER_SEC (1000000)

#define TDC7200_REG_ADR_CONFIG1                (0x00u)
#define TDC7200_REG_ADR_CONFIG2                (0x01u)
#define TDC7200_REG_ADR_INT_STATUS             (0x02u)
#define TDC7200_REG_ADR_INT_MASK               (0x03u)
#define TDC7200_REG_ADR_COARSE_CNTR_OVF_H      (0x04u)
#define TDC7200_REG_ADR_COARSE_CNTR_OVF_L      (0x05u)
#define TDC7200_REG_ADR_CLOCK_CNTR_OVF_H       (0x06u)
#define TDC7200_REG_ADR_CLOCK_CNTR_OVF_L       (0x07u)
#define TDC7200_REG_ADR_CLOCK_CNTR_STOP_MASK_H (0x08u)
#define TDC7200_REG_ADR_CLOCK_CNTR_STOP_MASK_L (0x09u)
#define TDC7200_REG_ADR_TIME1                  (0x10u)
#define TDC7200_REG_ADR_CLOCK_COUNT1           (0x11u)
#define TDC7200_REG_ADR_TIME2                  (0x12u)
#define TDC7200_REG_ADR_CLOCK_COUNT2           (0x13u)
#define TDC7200_REG_ADR_TIME3                  (0x14u)
#define TDC7200_REG_ADR_CLOCK_COUNT3           (0x15u)
#define TDC7200_REG_ADR_TIME4                  (0x16u)
#define TDC7200_REG_ADR_CLOCK_COUNT4           (0x17u)
#define TDC7200_REG_ADR_TIME5                  (0x18u)
#define TDC7200_REG_ADR_CLOCK_COUNT5           (0x19u)
#define TDC7200_REG_ADR_CLOCK_COUNTX(num)      (TDC7200_REG_ADR_CLOCK_COUNT1+2*((num)-1))
#define TDC7200_REG_ADR_TIME6                  (0x1Au)
#define TDC7200_REG_ADR_TIMEX(num)             (TDC7200_REG_ADR_TIME1+2*((num)-1))
#define TDC7200_REG_ADR_CALIBRATION1           (0x1Bu)
#define TDC7200_REG_ADR_CALIBRATION2           (0x1Cu)

#define TDC7200_REG_DEFAULTS_CONFIG2           (0x40u)      // reset defaults
#define TDC7200_REG_DEFAULTS_INT_MASK          (0x07u)      // reset defaults

#define TDC7200_REG_SHIFT_CONFIG1_FORCE_CAL    (7u)
#define TDC7200_REG_SHIFT_CONFIG1_PARITY_EN    (6u)
#define TDC7200_REG_SHIFT_CONFIG1_TRIGG_EDGE   (5u)
#define TDC7200_REG_SHIFT_CONFIG1_STOP_EDGE    (4u)
#define TDC7200_REG_SHIFT_CONFIG1_START_EDGE   (3u)
#define TDC7200_REG_SHIFT_CONFIG1_MEAS_MODE    (1u)
#define TDC7200_REG_SHIFT_CONFIG1_START_MEAS   (0u)

#define TDC7200_REG_VAL_CONFIG1_MEAS_MODE_MIN  (1u)
#define TDC7200_REG_VAL_CONFIG1_MEAS_MODE_MAX  (2u)
#define TDC7200_REG_VAL_CONFIG1_MEAS_MODE(num) ((num)-1)


#define TDC7200_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS  (6u)
#define TDC7200_REG_SHIFT_CONFIG2_AVG_CYCLES            (3u)
#define TDC7200_REG_SHIFT_CONFIG2_NUM_STOP              (0u)

#define TDC7200_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_2  (0u)
#define TDC7200_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_10 (1u)
#define TDC7200_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_20 (2u)
#define TDC7200_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_40 (3u)

#define TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MIN_VAL      (0u)
#define TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MIN          (1u << TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MIN_VAL)
#define TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MAX_VAL      (7u)
#define TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MAX          (1u << TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MAX_VAL)

#define TDC7200_REG_VAL_CONFIG2_NUM_STOP(num)           ((num)-1)
#define TDC7200_REG_VAL_CONFIG2_NUM_STOP_MAX            (5)


#define TDC7200_REG_SHIFT_INT_STATUS_MEAS_COMPLETE_FLAG  (4)
#define TDC7200_REG_SHIFT_INT_STATUS_MEAS_STARTED_FLAG   (3)
#define TDC7200_REG_SHIFT_INT_STATUS_CLOCK_CNTR_OVF_INT  (2)
#define TDC7200_REG_SHIFT_INT_STATUS_COARSE_CNTR_OVF_INT (1)
#define TDC7200_REG_SHIFT_INT_STATUS_NEW_MEAS_INT        (0)


#define TDC7200_REG_SHIFT_INT_MASK_CLOCK_CNTR_OVF_MASK   (2)
#define TDC7200_REG_SHIFT_INT_MASK_COARSE_CNTR_OVF_MASK  (1)
#define TDC7200_REG_SHIFT_INT_MASK_NEW_MEAS_MASK         (0)


TDC7200::TDC7200(const uint8_t pinEnable, const uint8_t pinCs, const uint32_t clockFreqHz)
    :   m_pinEnable(pinEnable),
        m_pinCs(pinCs),
        m_clkPeriodPs(uint64_t(PS_PER_SEC) / uint64_t(clockFreqHz)),
        m_overflowPs(0ull)
{
}

bool TDC7200::begin()
{
    // -- Enable TDC7200 - Reset to default configuration
    digitalWrite(m_pinEnable, LOW);
    pinMode(m_pinEnable, OUTPUT);
    // Disable for a short time
    delay(TDC7200_ENABLE_LOW_MS);

    // Enable and wait the maximum time after enabling LDO to assure VREG is stable.
    digitalWrite(m_pinEnable, HIGH);
    delay(TDC7200_ENABLE_T3_LDO_SET3_MS);

    // -- Configure SPI
    digitalWrite(m_pinCs, HIGH);
    pinMode(m_pinCs, OUTPUT);
    SPI.begin();

    // -- Comms sanity check
    if (   (spiReadReg8(TDC7200_REG_ADR_CONFIG2)  != TDC7200_REG_DEFAULTS_CONFIG2)
        or (spiReadReg8(TDC7200_REG_ADR_INT_MASK) != TDC7200_REG_DEFAULTS_INT_MASK) )
    {
        return false;
    }

    // Assert interrupt output on overflow and measurement finished
    spiWriteReg8(TDC7200_REG_ADR_INT_MASK,   bit(TDC7200_REG_SHIFT_INT_MASK_CLOCK_CNTR_OVF_MASK)
                                           | bit(TDC7200_REG_SHIFT_INT_MASK_COARSE_CNTR_OVF_MASK)
                                           | bit(TDC7200_REG_SHIFT_INT_MASK_NEW_MEAS_MASK) );

    return true;
}

bool TDC7200::setupMeasurement(const uint8_t cal2Periods, const uint8_t avgCycles, const uint8_t numStops, const uint8_t mode)
{
    uint8_t config2 = 0u;

    // Config2 Calibration2 periods
    if      (cal2Periods == 2)  config2 = TDC7200_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_2  << TDC7200_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS;
    else if (cal2Periods == 10) config2 = TDC7200_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_10 << TDC7200_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS;
    else if (cal2Periods == 20) config2 = TDC7200_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_20 << TDC7200_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS;
    else if (cal2Periods == 40) config2 = TDC7200_REG_VAL_CONFIG2_CALIBRATION2_PERIODS_40 << TDC7200_REG_SHIFT_CONFIG2_CALIBRATION2_PERIODS;
    else return false;
    m_cal2Periods = cal2Periods;

    // Config2 Avg Cycles
    uint8_t val = TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MIN_VAL;
    do {
        if ((1 << val) == avgCycles)
        {
            config2 |= val << TDC7200_REG_SHIFT_CONFIG2_AVG_CYCLES;
            break;
        }
        ++val;
    } while (val <= TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MAX_VAL);
    if (val > TDC7200_REG_VAL_CONFIG2_AVG_CYCLES_MAX_VAL)
        return false;

    // Config2 Num Stops
    if ((numStops == 0) or (numStops > TDC7200_REG_VAL_CONFIG2_NUM_STOP_MAX))
        return false;
   
    m_numStops = numStops;
    config2 |= TDC7200_REG_VAL_CONFIG2_NUM_STOP(numStops) << TDC7200_REG_SHIFT_CONFIG2_NUM_STOP;

    // Config1 Mode
    if ((mode < TDC7200_REG_VAL_CONFIG1_MEAS_MODE_MIN) or (mode > TDC7200_REG_VAL_CONFIG1_MEAS_MODE_MAX))
        return false;
    m_mode = mode;
    m_config1 = TDC7200_REG_VAL_CONFIG1_MEAS_MODE(mode) << TDC7200_REG_SHIFT_CONFIG1_MEAS_MODE;

    // Mode influences overflow, so update now.
    setupOverflow(m_overflowPs);

    // Config1 Start measurement
    m_config1 |= bit(TDC7200_REG_SHIFT_CONFIG1_START_MEAS);

    spiWriteReg8(TDC7200_REG_ADR_CONFIG2, config2);

    return true;     
}

void TDC7200::setupStopMask(const uint64_t stopMaskPs)
{
    // Convert duration of stopmask from [ps] to clock increments.
    uint16_t stopMaskClk = stopMaskPs / m_clkPeriodPs;
    spiWriteReg8(TDC7200_REG_ADR_CLOCK_CNTR_STOP_MASK_H, stopMaskClk >> 8);
    spiWriteReg8(TDC7200_REG_ADR_CLOCK_CNTR_STOP_MASK_L, stopMaskClk);
}

void TDC7200::setupOverflow(const uint64_t overflowPs)
{
    // Datasheet is rather vague on overflow configuration...
    // Some info from the net:
    //
    // https://e2e.ti.com/support/sensor/ultrasonic/f/991/p/445361/1905773?tisearch=e2e-sitesearch&keymatch=translation&pi239031350=2
    // For 8MHz clock:
    // In that case it depends on what's selected in the overflow registers (CLOCK_CNTR_OV_H/L registers
    // for mode 2, COARSE_CNTR_OV_H/L registers for mode 1). By default, they are set to 0xFFFF, which is
    // the maximum time out value. In mode 2, this corresponds to 8.192ms. In mode 1, the maximum time
    // out period corresponds to 454.164us.
    // --
    // For mode 1, all the counting is done using the internal time base (ring oscillator) which runs at 144MHz
    // (6.93ns). This translates to a timeout period of roughly 454.16us. Note you don't want to run Mode 1 for
    // long periods (> 500ns) as explained in Section 8.4.2.1 of the DS.
    //
    // https://e2e.ti.com/support/sensor/ultrasonic/f/991/p/475519/1710858
    // if I want to set a 300ns overflow in mode 1, how do I calculate the value should be written to COARSE_CNTR_OVF
    // You should use
    // COARSE_CNTR_OVF = 300ns / (normLSB * 63).
    // where 63 is the number of delay cells in the internal ring oscillator.
    // You can use the typical normLSB value of 55ps but if you want higher accuracy, then you can measure
    // normLSB (once every minute or so as your application permits) and update the COARSE_CNTR_OVF register.

    uint16_t coarseOvf = 0xFFFFu;   // For mode 1, maximum 454.164us
    uint16_t clockOvf  = 0xFFFFu;   // For mode 2, maximum 8.192ms

    // setupOverflow of 0 means as long as possible, so use to max. value.
    if (overflowPs)
    {
        if (1 == m_mode)
        {
            // Calculate number of ring oscillator ticks
            // Note: (TDC7200_RING_OSC_FREQ_HZ / PS_PER_SEC) == (TDC7200_RING_OSC_FREQ_MHZ / US_PER_SEC)
            const uint32_t ovf = (overflowPs * TDC7200_RING_OSC_FREQ_MHZ) / US_PER_SEC;
            // Clip to upper bound.
            if (ovf < 0xFFFFu)
            {
                coarseOvf = ovf;
            }
        }
        else
        {
            const uint32_t ovf = overflowPs / m_clkPeriodPs;
            // Clip to upper bound.
            if (ovf < 0xFFFFu)
            {
                clockOvf = ovf;
            }
        }
    }
    // Write both overflow for mode 1 & 2. If mode 1 is active, mode 2 will be set to max and vice versa.
    spiWriteReg8(TDC7200_REG_ADR_COARSE_CNTR_OVF_H, coarseOvf >> 8);
    spiWriteReg8(TDC7200_REG_ADR_COARSE_CNTR_OVF_L, coarseOvf);

    spiWriteReg8(TDC7200_REG_ADR_CLOCK_CNTR_OVF_H, clockOvf >> 8);
    spiWriteReg8(TDC7200_REG_ADR_CLOCK_CNTR_OVF_L, clockOvf);

    // Remember for mode changes
    m_overflowPs = overflowPs;
}


void TDC7200::startMeasurement()
{
    // Clear status
    spiWriteReg8(TDC7200_REG_ADR_INT_STATUS,   bit(TDC7200_REG_SHIFT_INT_STATUS_MEAS_COMPLETE_FLAG)
                                             | bit(TDC7200_REG_SHIFT_INT_STATUS_MEAS_STARTED_FLAG)
                                             | bit(TDC7200_REG_SHIFT_INT_STATUS_CLOCK_CNTR_OVF_INT)
                                             | bit(TDC7200_REG_SHIFT_INT_STATUS_COARSE_CNTR_OVF_INT)
                                             | bit(TDC7200_REG_SHIFT_INT_STATUS_NEW_MEAS_INT) );

    // Force recalculation of normLsb after measurement ended
    m_normLsb = 0ull;

    // Start measurement
    spiWriteReg8(TDC7200_REG_ADR_CONFIG1, m_config1);
}

bool TDC7200::readMeasurement(const uint8_t stop, uint64_t& tof)
{
    tof = 0ull;

    if (stop > m_numStops)
        return false;
/*
    uint8_t status = spiReadReg8(TDC7200_REG_ADR_INT_STATUS);
    if (not (status & (   bit(TDC7200_REG_SHIFT_INT_STATUS_MEAS_COMPLETE_FLAG))))
        return false;
*/
    // multiplier (2^shift) used to prevent rounding errors
    const uint8_t shift = 20;

    // Speed optimize: Cache normLsb for multiple stop tof calculations
    if (not m_normLsb)
    {
        const uint32_t calibration1 = spiReadReg24(TDC7200_REG_ADR_CALIBRATION1);
        const uint32_t calibration2 = spiReadReg24(TDC7200_REG_ADR_CALIBRATION2);

        // calCount scaled by 2^shift
        const int64_t calCount = ( int64_t(calibration2-calibration1) << shift ) / int64_t(m_cal2Periods - 1);

        // normLsb scaled by 2^shift, divided by calcount (scaled by 2^shift),
        // so multiply by 2^(2*shift) to compensate for divider in calCount
        m_normLsb  = (uint64_t(m_clkPeriodPs) << (2*shift)) / calCount;
    }

    switch (m_mode)
    {
        case 1:
        {
            const uint32_t timen        = spiReadReg24(TDC7200_REG_ADR_TIMEX(stop));          // TIME(n)
            tof = ( int64_t(timen) * m_normLsb ) >> shift;
            break;
        }
        case 2:
        {
            const uint32_t time1        = spiReadReg24(TDC7200_REG_ADR_TIME1);                // TIME1
            const uint32_t timen1       = spiReadReg24(TDC7200_REG_ADR_TIMEX(stop + 1));      // TIME(n+1)
            const uint32_t clockCountn  = spiReadReg24(TDC7200_REG_ADR_CLOCK_COUNTX(stop));   // CLOCK_COUNT(n)
            tof = ( (int64_t(time1) - int64_t(timen1)) * m_normLsb ) >> shift;
            tof += uint64_t(clockCountn) * uint64_t(m_clkPeriodPs);
            break;
        }
        default: return false;
    }
    // TOF for a pulses that didn't occur will be reported as all ones.
    // If this is the case, return 0 as value.
    if (not (~tof)) tof = 0ull;
    return true;
}

uint8_t TDC7200::spiReadReg8(const uint8_t addr)
{
    SPI.beginTransaction(SPISettings(TDC7200_SPI_CLK_MAX, TDC7200_SPI_ORDER, TDC7200_SPI_MODE));
    digitalWrite(m_pinCs, LOW);

    SPI.transfer((addr & TDC7200_SPI_REG_ADDR_MASK) | TDC7200_SPI_REG_READ);
    uint8_t val = SPI.transfer(0u);

    digitalWrite(m_pinCs, HIGH);
    SPI.endTransaction();

    return val;
}

uint32_t TDC7200::spiReadReg24(const uint8_t addr)
{
    SPI.beginTransaction(SPISettings(TDC7200_SPI_CLK_MAX, TDC7200_SPI_ORDER, TDC7200_SPI_MODE));
    digitalWrite(m_pinCs, LOW);

    SPI.transfer((addr & TDC7200_SPI_REG_ADDR_MASK) | TDC7200_SPI_REG_READ);
    uint32_t val;
    val = SPI.transfer(0u);
    val <<= 8;
    val |= SPI.transfer(0u);
    val <<= 8;
    val |= SPI.transfer(0u);

    digitalWrite(m_pinCs, HIGH);
    SPI.endTransaction();

    return val;
}

void TDC7200::spiWriteReg8(const uint8_t addr, const uint8_t val)
{
    SPI.beginTransaction(SPISettings(TDC7200_SPI_CLK_MAX, TDC7200_SPI_ORDER, TDC7200_SPI_MODE));
    digitalWrite(m_pinCs, LOW);

    (void)SPI.transfer16((((addr & TDC7200_SPI_REG_ADDR_MASK) | TDC7200_SPI_REG_WRITE) << 8) | val);

    digitalWrite(m_pinCs, HIGH);
    SPI.endTransaction();
}
