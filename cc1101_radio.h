#ifndef CC1101_RADIO_H
#define CC1101_RADIO_H

#include <linux/interrupt.h>
#include <linux/timer.h>

irqreturn_t cc1101_rx_interrupt(int irq, void *handle);
void cc1101_rx_timeout(struct timer_list *timer);
void cc1101_idle(cc1101_t* cc1101);
void cc1101_rx(cc1101_t* cc1101);
#ifndef RXONLY
void cc1101_tx(cc1101_t* cc1101, const char* buf, size_t len);
#endif
void cc1101_reset(cc1101_t* cc1101);

#endif