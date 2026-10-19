/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef TFA9873_Q6_H
#define TFA9873_Q6_H
struct q6afe_port;
int tfa9873_attach_afe(struct q6afe_port *port);
void tfa9873_detach_afe(struct q6afe_port *port);
int send_tfa_cal_in_band(void *buf, int size);
int send_tfa_cal_apr(void *buf, int size, bool read);
int send_tfa_cal_set_bypass(void *buf, int size);
int send_tfa_cal_set_tx_enable(void *buf, int size);
#endif
