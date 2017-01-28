//***************************************************************************************
//  MSP430 Blink the LED Demo - Software Toggle P1.4
//
//  Description; Toggle P1.4 by xor'ing P1.4 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP430i20xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.4|-->LED
//
//  J. Stevenson
//  Texas Instruments, Inc
//  July 2011
//  Built with Code Composer Studio v5
//***************************************************************************************

#include <msp430g2553.h>				

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
	P1DIR |= BIT4;					// Set P1.4 to output direction

	for(;;) {
		volatile unsigned int i;	// volatile to prevent optimization

		P1OUT ^= BIT4;				// Toggle P1.4 using exclusive-OR

		i = -1;					// SW Delay
		do i--;
		while(i != 0);
	}
	
	return 0;
}
