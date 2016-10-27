/*
 * vrs_cv5.h
 *
 *  Created on: 17. 10. 2016
 *      Author: Admin
 */

#ifndef VRS_CV5_H_
#define VRS_CV5_H_

uint32_t adc_init(void);
void led_init(void);
void nvic_init(void);
void usart_init(void);
int which_button(uint32_t AD_value);
void work(int buttonState);
void SendUSART2(char *s);

#endif /* VRS_CV5_H_ */
