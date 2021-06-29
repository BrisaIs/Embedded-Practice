/*
 * UART.c
 *
 *  Created on: Jan 24, 2021
 *      Author: Modutram
 */

#include "stm32f4xx.h"
#include "UART.h"
#include <stdio.h>
void uart2_init (void) {


    // Baud rate 115k2 with SYSCLK 84MHz (BRR = 45.5625)
    // USART2_BRR = (45 << 4) | 9;

    //USART2_CR1 = USART_CR1_UE | USART_CR1_TE;
/*1. Enable GPIOA clock by writing 1 to bit0 of AHB1ENR*/
RCC->AHB1ENR |= (1U<<0);
/*2. Enable USART2 clock by writing 1 to bit17 of APB1ENR*/
RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable peripheral clock for USART2
/*3. Enable alt7 for USART2 */
GPIOA->AFR[0] |= 0x7700;
/*4. Enable alternate function for PA2, PA3 */
GPIOA->MODER |= 0b1010 << 4;
/*5. Set UART: 9600 baud @ 16 MHz */
USART2->BRR =  0x0683;
/*6. Enable TX, RX, 8-bit data */
USART2->CR1 = 0x000C;
/*7. Set UART :1 stop bit */
USART2->CR2 = 0;
/*8. Set UART: No flow control */
USART2->CR3 = 0;
/*9. Enable USART2 */
USART2->CR1 |= 0x2000;
GPIOA->OSPEEDR |= 0b1111 << 4; // PA2, PA3 to high speed
}
/* Write a character to USART2 */
int uart2_write (int ch) {
/*10. wait until Tx buffer empty*/
while (!(USART2->SR & 0x0080)) {}
/*11. Write character to DR */
USART2->DR = (ch & 0xFF);
return ch;
}
/* Read a character from USART2 */
int uart2_read(void) {
/*12. Wait until character arrives*/
while (!(USART2->SR & 0x0020)) {}
/*13. Return the received character */
return USART2->DR;
}
/*Interface to the c standard I/O library*/
struct __FILE { int handle; };
FILE __stdin = {0};
FILE __stdout = {1};
FILE __stderr = {2};
/*fgetc is called by c library console input.
The function will echo the character received*/
int fgetc(FILE *f) {
int c;
/*1. read the character from console */
/*2. If '\r', after it is echoed, a '\n' is appended*/
if (c == '\r') {
uart2_write(c); /* echo */
c = '\n';
}
/*3. Echoe*/
uart2_write(c);
/*4. Return character*/
return c;
}
/*fputc is called by c library console output. */
int fputc (int c, FILE *f) {
/*5. Write the character to console */
return uart2_write(c);
}
