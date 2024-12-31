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
//uint32_t result;

//uint32_t text = "h";

//const char *text = "hello world\n"; // Include the newline for formatting
	// ADC interrupt handler
	void ADC_Handler(void) {
		uint32_t status = adc_get_status(ADC);

		if (status & ADC_ISR_EOC0) { // Check if End of Conversion for channel 0
			uint16_t value = adc_get_channel_value(ADC, ADC_CHANNEL_0); // Read ADC result
			DEBUG_INFO("ADC Value: %d\n", value); // Print via USB CDC
		}
	}
  //void ADC_IrqHandler(void)
  //{
	  //// Check the ADC conversion status
	  //if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY)
	  //{
		  //// Get latest digital data value from ADC and can be used by application
		  //result = adc_get_latest_value(ADC);
		 //
	  //}
  //}
  void adc_setup(void)
  {
	  uint32_t ADC_CLOCK = sysclk_get_main_hz()/8 ;
	  
	  adc_init(ADC, sysclk_get_main_hz(), ADC_FREQ_MAX, ADC_STARTUP_FAST);

	  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);

	  adc_set_resolution(ADC, ADC_MR_LOWRES_BITS_12);

	  adc_enable_channel(ADC, ADC_CHANNEL_5);

	  adc_enable_interrupt(ADC, ADC_IER_EOC0);

	  adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
  }

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	SystemInit();
	board_init();
	udc_start();

delay_init();
//pmc_enable_periph_clk(ID_PIOB);

//pmc_enable_periph_clk(ID_PIOD); // clock enable for port D
//pmc_enable_periph_clk(ID_PIOA);
//pmc_enable_periph_clk(ID_ADC); // clock enable for ADC

//uint16_t tms = 500;
//uint32_t buff[1];
//buff[0];

//pio_set_output(PIOB, PIO_PB27, LOW, DISABLE, ENABLE);
//pio_set_output(PIOB, PIO_PB27, HIGH, DISABLE, ENABLE);

//pio_set_output(PIOD, PIO_PD8, LOW, DISABLE, ENABLE);

//irq_initialize_vectors();
//cpu_irq_enable();
//NVIC_EnableIRQ(ADC_IRQn); 
//adc_setup();
//adc_start(ADC);
//pio_configure_pin(ID_PIOD, PIO_OUTPUT_1);
DEBUG_INFO("example, started!\n");
//ADC_IrqHandler();
//adc_enable_interrupt(ADC, ADC_IER_EOC0);
//NVIC_EnableIRQ(ADC_IRQn); 
//adc_set_trigger(ADC, ADC_TRIG_TIO_CH0);

//adc_start();
//configure_led();
configure_timer();
while(1)
{
	
	
		//uint16_t value = 1234;
		//DEBUG_INFO("Value: %d\n", value); // Output: "Value: 1234"
		DEBUG_INFO("Hello World!\n");
	//pio_toggle_pin(PIO_PD8);
	//delay_ms(500);
	//adc_start(ADC);
	//pio_set_output(PIOB, PIO_PB27, LOW, DISABLE, ENABLE);
	//delay_ms(500);
	//pio_set_output(PIOB, PIO_PB27, HIGH, DISABLE, ENABLE);
	//delay_ms(500);
	//udi_cdc_read_no_polling(buff,1);
	//if(buff[0]==49)
	//{
		//tms = 100;
		//buff[0] = 0;
		//udi_cdc_putc(50);
		//
	//}
	//pio_set_output(PIOD, PIO_PD8, LOW, DISABLE, ENABLE);
	//delay_ms(tms);
	//pio_set_output(PIOD, PIO_PD8, HIGH, DISABLE, ENABLE);
	//delay_ms(tms);
	
	//ADC_IrqHandler();
	//DEBUG_INFO("value: %d\n", result);
	//char buffer1[20];
	//sprintf(buffer1, "%d", (uint16_t)result);
	//strcat(buffer1, "\n");
	//for(int i=0; i<strlen(buffer1);i++)
	//{
		//udi_cdc_putc(buffer1[i]);
		//
	//}
	//printf("hello world\n");
	//sprintf("hello world\n");
	
	//sprintf(buffer1, "%d", (uint16_t)text);
	//strcat(buffer1, "\n");
	//for(int i=0; i<strlen(buffer1);i++)
	//{
		//udi_cdc_putc(buffer1[i]);
		//
	//}
	
	
	//char buffer1[16];
	//int len = sprintf(buffer1, "%d\n", (uint16_t)text);
	//udi_cdc_write_buf(buffer1, len);



	
	//udi_cdc_write_buf(text, strlen(text));
	//udi_cdc_putc(48);
	//udi_cdc_putc(10);
	//DEBUG_INFO("Value: %d\n", value);
	delay_ms(500);
	
}

	/* Insert application code here, after the board has been initialized. */
}
