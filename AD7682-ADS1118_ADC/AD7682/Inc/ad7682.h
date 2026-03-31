#ifndef AD7682_H
#define AD7682_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"


#define AD7682_VREF        3.3f

#define AD7682_LSB         (AD7682_VREF / 65536.0f)

#define AD7682_TX_IN0      ((uint16_t)0xE1E4)
#define AD7682_TX_IN1      ((uint16_t)0xE3E4)

/* --- Driver handle ------------------------------------------------------- */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cnv_port;
    uint16_t           cnv_pin;
} AD7682_Handle;


void AD7682_Init(AD7682_Handle *h, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cnv_port, uint16_t cnv_pin);


void AD7682_ReadBoth(AD7682_Handle *h,
                     float    *v0,   uint16_t *raw0,
                     float    *v1,   uint16_t *raw1);

#ifdef __cplusplus
}
#endif
#endif /* AD7682_H */
