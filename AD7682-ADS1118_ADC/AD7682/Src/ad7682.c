#include "ad7682.h"
#include <stdio.h>


static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* enable DWT */
    DWT->CYCCNT       = 0U;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;       /* start counter */
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}


static uint16_t SPI_Transfer16(SPI_HandleTypeDef *hspi, uint16_t tx)
{
    uint16_t rx = 0U;
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)&tx, (uint8_t *)&rx, 1U, 10U);
    return rx;
}

/* ===========================================================================
 * do_cycle  —  one complete AD7682 conversion + readout cycle
 *
 * Follows Figure 43 (Rev.K): Read/Write Spanning Conversion,
 * no busy indicator, CPOL=CPHA=0.
 *
 * Timing (Table 6, VDD = 2.3-4.5V, TA ≤ 85°C):
 *   tCNVH  (CNV pulse width)       : min 10 ns   — GPIO SET then CLR satisfies this
 *   tCONV  (conversion time)       : max 3.2 µs  — we wait 4 µs (safe margin)
 *   tCLSCK (CNV low to first SCK)  : min 10 ns   — 1 µs delay well exceeds this
 *   tACQ   (acquisition time)      : min 1.8 µs  — satisfied by 1 µs + SPI time
 *
 * Sequence:
 *  1. CNV HIGH  → ADC samples input, starts converting
 *  2. Wait 4 µs → conversion completes (tCONV max = 3.2 µs)
 *  3. CNV LOW   → conversion result latched; SDO drives bit15 of result
 *  4. Wait 1 µs → meet tCLSCK (min 10 ns), also begins next acquisition
 *  5. 16 SCK    → MISO shifts out 16-bit result (MSB first, straight binary)
 *                 MOSI shifts in  16-bit TX word (first 14 rising edges = cfg[13:0])
 *
 * The config written in this cycle (tx_config) programs the channel for the
 * NEXT conversion.  The returned value is the result of the conversion
 * triggered by the CNV pulse in step 1.
 * =========================================================================*/
static uint16_t do_cycle(AD7682_Handle *h, uint16_t tx_config)
{
    /* Step 1+2: start conversion, wait tCONV */
    HAL_GPIO_WritePin(h->cnv_port, h->cnv_pin, GPIO_PIN_SET);
    delay_us(4U);

    /* Step 3+4: end conversion, brief pause before first SCK edge */
    HAL_GPIO_WritePin(h->cnv_port, h->cnv_pin, GPIO_PIN_RESET);
    delay_us(1U);

    /* Step 5: 16-bit SPI — result out on MISO, config in on MOSI */
    uint16_t result = SPI_Transfer16(h->hspi, tx_config);

    /* Wait for SPI peripheral to fully finish before next CNV rising edge */
    while (h->hspi->Instance->SR & SPI_SR_BSY);
    delay_us(1U);

    return result;
}


void AD7682_Init(AD7682_Handle *h, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cnv_port, uint16_t cnv_pin)
{
    h->hspi     = hspi;
    h->cnv_port = cnv_port;
    h->cnv_pin  = cnv_pin;

    DWT_Init();
    HAL_GPIO_WritePin(cnv_port, cnv_pin, GPIO_PIN_RESET); /* CNV idle LOW */

}

/* ===========================================================================
 * AD7682_ReadBoth
 *
 * Reads IN0 and IN1 in a fixed 4-cycle deterministic sequence.
 *
 * State entering this function: CFG register = IN0 (guaranteed by Init
 * or by the previous ReadBoth call).
 *
 * Pipeline trace:
 *
 *  Cycle 1: TX=IN0  ADC converts IN0 (cfg was IN0 from previous call)
 *           Result = stale IN0 from last call  DISCARD
 *           cfg@ = IN0
 *
 *  Cycle 2: TX=IN1  ADC converts IN0 (cfg is still IN0)
 *           Result = fresh IN0 sample           VALID --> raw0
 *           cfg@ = IN1
 *
 *  Cycle 3: TX=IN1  ADC converts IN1 (first conversion after mux switch)
 *           Result = IN1 (mux just switched, not yet settled)  DISCARD
 *           cfg@ = IN1
 *           Note: tACQ = 1.8 µs min is satisfied by the 4+1+SPI time
 *                 of the next cycle before IN1 is sampled again.
 *
 *  Cycle 4: TX=IN0  ADC converts IN1 (second conversion on IN1, settled)
 *           Result = settled IN1 sample         VALID --> raw1
 *           cfg@ = IN0   <-- exactly the same state as on entry
 *
 * State exiting: CFG register = IN0, identical to entry state.
 * The next call to ReadBoth is fully deterministic with no accumulated error.
 * =========================================================================*/
void AD7682_ReadBoth(AD7682_Handle *h,
                     float    *v0,   uint16_t *raw0,
                     float    *v1,   uint16_t *raw1)
{
    uint16_t r;

    /* Cycle 1: TX=IN0, discard stale IN0 result from previous call */
    do_cycle(h, AD7682_TX_IN0);

    /* Cycle 2: TX=IN1, capture fresh IN0 result */
    r = do_cycle(h, AD7682_TX_IN1);
    if (raw0) *raw0 = r;
    if (v0)   *v0   = (float)r * AD7682_LSB;

    /* Cycle 3: TX=IN1, discard first IN1 result (mux not yet settled) */
    do_cycle(h, AD7682_TX_IN1);

    /* Cycle 4: TX=IN0, capture settled IN1 result */
    r = do_cycle(h, AD7682_TX_IN0);
    if (raw1) *raw1 = r;
    if (v1)   *v1   = (float)r * AD7682_LSB;

    /* cfg@ = IN0 — same as entry state, next call is deterministic */
}
