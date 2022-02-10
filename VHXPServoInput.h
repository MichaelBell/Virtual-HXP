#pragma once

#ifdef __cplusplus
extern "C" {
#endif
void servo_input_start();
uint16_t servo_get_on_duration(int board, int idx);
uint16_t servo_get_off_duration(int board, int idx);
#ifdef __cplusplus
}
#endif
