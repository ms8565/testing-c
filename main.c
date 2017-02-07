#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"

#include <string.h>
#include <stdio.h>

#define NUM_BUCKETS (101)
#define NUM_MEASUREMENTS (1000)
#define MIN_LOWER_LIMIT (50)
#define MAX_LOWER_LIMIT (9950)
#define DEFAULT_LOWER_LIMIT (950)

unsigned int wasPost = 0;
uint8_t buffer[BufferSize];
unsigned int rising_edge_count = 0;
unsigned int update_SM = 0;
// Initalize this to 1 as when initalizing to 0 and clearing some elements are non-zero
// Possibly a compiler bug??? Changing optimatization level from -o0 to default elmiinates the issue
uint16_t pulse_time_hist[NUM_BUCKETS] = {0};

typedef enum {
    STATE_POST,
	STATE_POST_FAIL_PROMPT,
	STATE_PARSE_LIMITS,
	STATE_PERFORM_MEASUREMENTS,
	STATE_DISPLAY_HIST,
    STATE_FAULT,
} state_t;

typedef enum {
    EVENT_POST_COMPLETE,
    EVENT_RISING_EDGE_DETECT,
    EVENT_RERUN_POST,
    EVENT_START_MEASUREMENTS,
    EVENT_HIST_DISP_DONE,
    EVENT_MAX_EVENT
} event_t;

state_t state = STATE_POST;

void processEvent(event_t event) {
    switch(event) {
        // 100 ms post time is over
        case (EVENT_POST_COMPLETE):
            switch (state) {
                case(STATE_POST):
                    if (rising_edge_count == 0) {
                        state = STATE_POST_FAIL_PROMPT;
                    } else {
                        state = STATE_PARSE_LIMITS;
                    }
                    update_SM = 1;
                    break;
                default:
                    update_SM = 1;
                    state = STATE_FAULT;
            } // end state
            break;  // end EVENT_POST_COMPLETE
        
        case (EVENT_RISING_EDGE_DETECT):
            switch (state) {
                case (STATE_POST):
                    rising_edge_count += 1;
                    break;
                case (STATE_PERFORM_MEASUREMENTS):
                    rising_edge_count += 1;
                    if (rising_edge_count == NUM_MEASUREMENTS) {
                        state = STATE_DISPLAY_HIST;
                        update_SM = 1;
                    } 
                    // Store measurement Here
                    break;
            } // end state
            break;  // end EVENT_POST_COMPLETE
        
        case (EVENT_RERUN_POST):
            switch (state) {
                case(STATE_POST_FAIL_PROMPT):
                    state = STATE_POST;
                    update_SM = 1;
                    break;
                default:
                    update_SM = 1;
                    state = STATE_FAULT;
            } // end state
            break; // end EVENT_RERUN_POST
        
        case (EVENT_START_MEASUREMENTS):
            switch (state) {
                case (STATE_PARSE_LIMITS) :
                    update_SM = 1;
                    rising_edge_count = 0;
                    state = STATE_PERFORM_MEASUREMENTS;
                    break;
                default:
                    update_SM = 1;
                    state = STATE_FAULT;
            } // end state
            break;  // end EVENT_START_MEASUREMENTS
            
        case (EVENT_HIST_DISP_DONE):
            switch(state) {
                case (STATE_DISPLAY_HIST):
                    update_SM = 1;
                    state = STATE_PARSE_LIMITS;
                    break;
                default:
                    update_SM = 1;
                    state = STATE_FAULT;
            } // end state
            break; // end EVENT_HIST_DISP_DONE
            
        default:
            update_SM = 1;
            state = STATE_FAULT;
    }
}

void EXTI0_IRQHandler(void) {
    // Center Button
    if ((EXTI->PR1 & 1) == 1) {
        EXTI->EMR1 = 0x1;
        processEvent(EVENT_RISING_EDGE_DETECT);
        Green_LED_Toggle();
        EXTI->PR1 = 1;    
        EXTI->EMR1 = 0x0;
	}
}

void TIM6_DAC_IRQHandler(void) {

    if((TIM6->SR & 1) != 0)    {                  // If update flag is set
		TIM6->DIER &= ~1;  
        TIM6->CR1 &= ~TIM_CR1_CEN;
        processEvent(EVENT_POST_COMPLETE);
        TIM6->SR &= ~TIM_SR_UIF;                            // Interrupt has been handled
    }
}

void timer_Init() {
    // Enable the clock for timer 6
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    // Configure timer to count at 2 KHz -> 500 us
    // System clock = 80 MHz / (4000) =  20 kHz
    TIM6->PSC = 39999;
	// Set Overflow to 200 * 500 us = 100 ms 
    TIM6->ARR = 4000;//200;
	// Trigger a write of registers
    TIM6->CR1 |= 4; // Prevent UIF from firing on an EGR write
    TIM6->EGR |= 1; 
    TIM6->DIER |= 1;
    // DIER write is setting UIF flag, clear it so we don't go into interrupt as soon as timer goes on
    //TIM6->SR &= ~TIM_SR_UIF;
    
}

void startFastTimer() {
    // Configure timer to count at 1 MHz -> 1 us
    // System clock = 80 MHz / (80) =  1 MHz
    TIM6->PSC = 79;
	// Set Overflow maximum value to 65535 * 1 us = 65.535 ms 
    TIM6->ARR = 0xffff;
    TIM6->EGR |= 1; 
}

void clearHist() {
    unsigned int i;
    for (i = 0; i < NUM_BUCKETS; i++) {
        pulse_time_hist[i] = 0;
    }
}

void printHist() {
    unsigned int i;
    int n;
    unsigned int count = 0;
    
    n = sprintf((char *)buffer, "Displaying Histogram: \r\n");
    USART_Write(USART2, buffer, n);
    for (i = 0; i < NUM_BUCKETS; i++) {
        count = pulse_time_hist[i];
        if (count != 0) {
            n = sprintf((char *)buffer, "Index: %u Count: %u\r\n", i,count);
            USART_Write(USART2, buffer, n);
        }
    }
}

int parseLowerLimit() {
    uint8_t rx_arr[10];
    unsigned int value;
    unsigned int length = 0;
    int n;
    
    while (1) {
        rx_arr[length] = USART_Read(USART2);
        if ((rx_arr[length]) == 0x0d) {
            if (length == 0) {
                // Write a new line after echoing user input
                n = sprintf((char *)buffer, "\r\n");
                USART_Write(USART2, buffer, n);
                return DEFAULT_LOWER_LIMIT;
            }
            break;
        }
        // Echo back what the user types
        n = sprintf((char *)buffer, "%c",rx_arr[length]);
        USART_Write(USART2, buffer, n);
        length += 1;
    }
    
	value = atoi(rx_arr);
    // Write a new line after echoing user input
    n = sprintf((char *)buffer, "\r\n");
    USART_Write(USART2, buffer, n);
    return value;
}

int validLowerLimit(int limit) {
    if ((limit >= MIN_LOWER_LIMIT) && (limit <= MAX_LOWER_LIMIT)) {
        return 1;
    } else {
        return 0;
    }
}

int main(void){
	int		n ;
    int limit = 0;
    
	System_Clock_Init();
    
    // Enable clk to PortA
    #warning MOVE oneoff statements into an init function
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    
	LED_Init();
	UART2_Init();
    timer_Init();
    
    // Enable interrupt vectors
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);
		
	
	// Configure PA0, PA1, PA2, PA3, PA5 as pull-down
	GPIOA->PUPDR &= ~0xCFF;
	GPIOA->PUPDR |= 0x8AA;
	
	// Set interrupt to be rising edge sensitive
	EXTI->RTSR1 |= EXTI_RTSR1_RT0;
	
	
		// Set PA0, PA1, PA2, PA3, and PA5 as input
	GPIOA->MODER &= ~0xCFF;

    // Kick of a state machine update event
    rising_edge_count = 0;
	update_SM = 1;
	state = STATE_POST;
	
	while (1){
        if (update_SM == 1) { 
            switch (state) {
                case (STATE_POST):
                    update_SM = 0;
                    EXTI->IMR1 |= EXTI_IMR1_IM0;
                    // Enable Interrupts for timer 6
                    TIM6->DIER |= 1;  
                    TIM6->CR1 |= TIM_CR1_CEN;
					
                    Red_LED_On();
                    Green_LED_Off();
                    n = sprintf((char *)buffer, "Running POST!\r\n");
                    USART_Write(USART2, buffer, n);
                    wasPost = 1;                    
                    break;
                    
                case (STATE_POST_FAIL_PROMPT):
					// Mask off rising edge interrupts
                    EXTI->IMR1 &= ~EXTI_IMR1_IM0;
                    Red_LED_Off();
                    Green_LED_Off();
                    n = sprintf((char *)buffer, "POST Failed\r\n");
                    n += sprintf((char *)buffer+n, "Press ENTER to re-run POST...\r\n");
                    USART_Write(USART2, buffer, n);	
                    while (USART_Read(USART2) != 0x0d);
                    processEvent(EVENT_RERUN_POST);
                    break;
                    
                case (STATE_PARSE_LIMITS):
                    // Mask off rising edge interrupt
                    EXTI->IMR1 &= ~EXTI_IMR1_IM0;
                    Red_LED_Off();
                    Green_LED_On();
                    if (wasPost == 1) {
                        n = sprintf((char *)buffer, "Post PASSED!\r\n");
                        USART_Write(USART2, buffer, n);	
                    }
                    n = sprintf((char *)buffer, "%u edges detected.\r\n", rising_edge_count);
                    // remove?
					rising_edge_count = 0;
                    USART_Write(USART2, buffer, n);	
                    while(validLowerLimit(limit) == 0) {
                        n = sprintf((char *)buffer, "Enter lower limit in range of 50-9950 microseconds and press ENTER: ");
                        USART_Write(USART2, buffer, n);	
                        limit = parseLowerLimit();
                    }
                    n = sprintf((char *)buffer, "Starting with lower limit %d\r\n", limit);
                    USART_Write(USART2, buffer, n);	
                    processEvent(EVENT_START_MEASUREMENTS);
                    break;
                    
                case (STATE_PERFORM_MEASUREMENTS):
                    Red_LED_Off();
                    Green_LED_Off();
                    clearHist();
                    n = sprintf((char *)buffer, "1000 Measurements in progress...\r\n");
                    USART_Write(USART2, buffer, n);	
                    startFastTimer();
                    EXTI->IMR1 |= EXTI_IMR1_IM0;
                    update_SM = 0;
                    // Enable interrupts rising edge
                    //state = STATE_DISPLAY_HIST;
                    break;
                    
                case (STATE_DISPLAY_HIST):
                    EXTI->IMR1 &= EXTI_IMR1_IM0;
                    printHist();
                    processEvent(EVENT_HIST_DISP_DONE);
                    wasPost = 0;
                    break;
                    
                case (STATE_FAULT):    
                default:
                    Red_LED_On();
                    Green_LED_On();
                    n = sprintf((char *)buffer, "FAULT STATE\r\n");
                    USART_Write(USART2, buffer, n);	
                    update_SM = 0;
                
            } // end case
        }
    }
}

