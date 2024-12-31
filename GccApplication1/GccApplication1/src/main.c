/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include "string.h"
#include "stdio.h"

 #define LED_PIN PIO_PB27
 #define LED_PIN_MASK (1 << 27)

// Define the DEBUG_INFO macro
#define DEBUG_INFO(format, ...) \
do { \
	char buffer[128]; /* Adjust size if needed */ \
	int len = snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
	debug_print(buffer, len); \
} while (0)

// Function to send the buffer via USB CDC
void debug_print(const char *message, int length) {
	for (int i = 0; i < length; i++) {
		udi_cdc_putc(message[i]);
	}
}

// Timer Counter interrupt handler
void TC0_Handler(void) {
    // Clear the interrupt status to acknowledge
    uint32_t status = tc_get_status(TC0, 0);

    if (status & TC_SR_CPCS) { // Check if the RC Compare event occurred
        //pio_toggle_pin(LED_PIN); // Toggle the LED state
		DEBUG_INFO("TC0 handler\n");
    }
}

// Configure the Timer Counter to generate periodic interrupts
void configure_timer(void) {
    // Enable Timer Counter peripheral clock
    pmc_enable_periph_clk(ID_TC0);

    // Configure the Timer Counter (TC0 Channel 0)
    tc_init(TC0, 0, TC_CMR_TCCLKS_TIMER_CLOCK2 | // Timer clock = MCK/8
                      TC_CMR_WAVE |              // Waveform mode
                      TC_CMR_WAVSEL_UP_RC);      // Count up to RC and reset

    // Set the frequency of the timer (example: 1 Hz for LED blinking)
    uint32_t timer_clock = sysclk_get_peripheral_hz() / 8; // MCK/8
    uint32_t rc_value = timer_clock / 2; // RC value for 0.5 Hz (1 second period)
    tc_write_rc(TC0, 0, rc_value);

    // Enable the RC Compare interrupt
    tc_enable_interrupt(TC0, 0, TC_IER_CPCS);

    // Enable the Timer Counter interrupt in the NVIC
    NVIC_EnableIRQ(TC0_IRQn);

    // Start the timer
    tc_start(TC0, 0);
}

// Configure the LED pin as output
void configure_led(void) {
    // Enable the PIO clock for the LED pin
    pmc_enable_periph_clk(ID_PIOB);

    // Configure the pin as an output
    pio_configure(PIOB, PIO_OUTPUT_0, LED_PIN_MASK, PIO_DEFAULT);

    // Set the LED initially off
    pio_clear(PIOB, LED_PIN_MASK);
}


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	SystemInit();
	board_init();
	udc_start();

delay_init();

DEBUG_INFO("example, started!\n");

configure_timer();
while(1)
{
	
		DEBUG_INFO("Hello World!\n");
	
	delay_ms(500);
	
}

	/* Insert application code here, after the board has been initialized. */
}
