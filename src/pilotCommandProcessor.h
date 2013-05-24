#pragma once

extern int16_t TX_roll, TX_pitch, TX_throttle, TX_yaw, TX_AUX1, TX_AUX2, TX_AUX3, TX_AUX4;
extern int16_t TX_AUX5, TX_AUX6, TX_AUX7, TX_AUX8, TX_AUX9, TX_AUX10, TX_AUX11, TX_AUX12;
extern uint64_t AUX_chan_mask;
extern bool throttlePanic;

void processPilotCommands();