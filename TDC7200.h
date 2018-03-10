#pragma once
#include <inttypes.h>

class TDC7200
{
    public:
        /**
         * Constructor.
         * @param[in] pinEnable   Mcu pin controlling TDC7200 enable input.
         * @param[in] pinCs       Mcu pin controlling TDC7200 SPI CSB input.
         * @param[in] clockFreqHz Clock frequency supplied at TDC7200 clock input, range [1..16] [MHz].
         */
        TDC7200(const uint8_t pinEnable, const uint8_t pinCs, const uint32_t clockFreqHz);

        /**
         * Initialize TDC7200.
         */
        bool begin();

        /**
         * Setup measurement parameters.
         * @param[in] cal2Periods Set calibration2 periods [2,10,20,40].
         * @param[in] avgCycles   Set number of averaging cycles [1,2,4,8,16,32,64,128]. 
         * @param[in] numStops    Set number of stop pulses [1..5].
         * @param[in] mode        Set measurement mode [1,2]. Mode 1 is for measurements <500 [ns].
         * @return                True, when all parameters were valid and setup succeeded.
         */
        bool setupMeasurement(const uint8_t cal2Periods, const uint8_t avgCycles, const uint8_t numStops, const uint8_t mode);

        /**
         * Setup stop mask.
         * @param[in] stopMaskPs  Duration of stop mask, in [ps]. Will be rounded to number of external
         *                        clock counts, so actual value used might be slightly different.
         */
        void setupStopMask(const uint64_t stopMaskPs);

        /**
         * Setup overflow time.
         * @param[in] overflowPs  Overflow time, in [ps]. If tof takes longer than overflowPs,
         *                        a timeout will occur.
         * @remark Currently only functional for mode2, as formula for mode1 doesn't seem correct.
         */
        void setupOverflow(const uint64_t overflowPs);

        /**
         * Start a new measurement.
         */
        void startMeasurement();

        /**
         * Read measurement result.
         * Caller must make sure sufficient time has elapsed for all pulses to occur before calling this function.
         * @param[in]  stop       Index of stop pulse to read time for, [1..numStops].
         * @param[out] tof        Measured time-of-flight from start pulse to given stop pulse, or 0 when
         *                        pulse wasn't recorded (didn't occur, or not within overflow time).
         * @return                True, when all parameters were valid and time-of-flight was retrieved.
         */
        bool readMeasurement(const uint8_t stop, uint64_t& tof);

    private:
        uint8_t  m_pinEnable;       //< Mcu pin controlling TDC7200 enable input.
        uint8_t  m_pinCs;           //< Mcu pin controlling TDC7200 SPI CSB input.
        uint32_t m_clkPeriodPs;     //< Clock period in [ps].
        uint8_t  m_cal2Periods;     //< Calibration2, number of measuring clock periods, one of [2,10,20,40].
        uint8_t  m_config1;         //< CONFIG1 register value, used to start measurement.
        uint8_t  m_mode;            //< Measurement mode [1,2].
        uint8_t  m_numStops;        //< Number of stops per measurement.
        int64_t  m_normLsb;         //< Cached normLsb value for tof calculation.
        uint64_t m_overflowPs;      //< Overflow time, in [ps].

        uint8_t  spiReadReg8(const uint8_t addr);
        uint32_t spiReadReg24(const uint8_t addr);
        void     spiWriteReg8(const uint8_t addr, const uint8_t val);
};

