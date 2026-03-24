#pragma once
#include <cstdint>  // for uint32_t, uint16_t, uint8_t

#ifdef __cplusplus
extern "C" {
#endif

// C linkage for Wit C SDK callbacks
void minimal_dog_sensor_update(uint32_t uiReg, uint32_t uiRegNum);
void minimal_dog_delay_ms(uint16_t ucMs);
void minimal_dog_serial_write(uint8_t* p_ucData, uint32_t uiLen);

#ifdef __cplusplus
}
#endif
