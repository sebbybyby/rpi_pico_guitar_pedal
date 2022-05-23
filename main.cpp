/**
 * @file main.cpp
 * @author Sebastian Kowalski (c18325906)
 * @brief 
 * 
 *          Written by Sebastian Kowalski as part of final year project
 *          for DT021a/4 (2022). 
 * 
 *          This code makes the microcontroller do the guitar stuff
 * 
 *          PICO-SSD1306 library (used for display) by Harby:
 *          https://github.com/Harbys/pico-ssd1306
 * 
 * @version pre-alpha 0.000000000000000000000001
 * @date 2022-05-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "ssd1306.h"
#include "include/distortion_map.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "pico-ssd1306/textRenderer/8x8_font.h"
#include "pico/float.h"

// ADC and PWM pin numbers
#define ADC_PIN 26          // Input AtoD pin
#define PWM_PIN 16          // Output PWM pin

// i2c screen hardware defines
#define I2C_PIN_SDA 0           // i2c data pin
#define I2C_PIN_SCL 1           // i2c clock pin
#define I2C_PORT i2c0           // SSD1306 i2c port
#define I2C_SCREEN_ADDRESS 0x3c // SSD1306 i2c address

// LED pin numbers
#define CLIP_LED 2
#define OUTPUT_LED 4
#define PLAY_LED 3
#define ONBOARD_LED  25

// Button pin numbers
#define BUTTON_LEFT 5
#define BUTTON_RIGHT 9
#define BUTTON_UP 7
#define BUTTON_DOWN 8
#define BUTTON_CENTRE 6

// 32-bit GPIO register values equivalent to button press
#define BUTTON_LEFT_PRESSED 0x20
#define BUTTON_RIGHT_PRESSED 0x200
#define BUTTON_UP_PRESSED 0x80
#define BUTTON_DOWN_PRESSED 0x100
#define BUTTON_CENTRE_PRESSED 0x40

// Mask so that only relevant button GPIO pins are considered
#define GPIO_BUTTON_MASK 0x3E0

// Yes. Very last minute work absolutely happened.
#define PROGRAM_VERSION "v23_05_22" 

// Priority flags for RP2040 interrupts
#define IRQ_LOW_PRIORITY 0xc0
#define IRQ_DEFAULT_PRIORITY 0x80
#define IRQ_HIGH_PRIORITY 0x40
#define IRQ_MAX_PRIORITY 0x00

// Constants required for calculating low-pass coefficients
// Floats ARE used for this as the code is only executed once on triggering
// a re-sync update by the user, stays far far away from ADC / PWM interrupts
#define PI_2 6.28318530718
#define PI 3.14159265359
#define ROOT_2 1.41421356237
#define SAMPLING_PERIOD 2.2e-05
#define HALF_SAMPLING_PERIOD 1.1e-05
#define EXPONENT_SIZE 16384
#define LOWPASS_BUFFER_SIZE 8


// Prototypes
bool timer_callback(struct repeating_timer *t); 
struct sample_buffer;

/* =================================================================================
~ 
~    _____   __   __              _    ______          __   __                  
~   |  ___| / _| / _|            | |   | ___ \        / _| / _|                 
~   | |__  | |_ | |_   ___   ___ | |_  | |_/ / _   _ | |_ | |_   ___  _ __  ___ 
~   |  __| |  _||  _| / _ \ / __|| __| | ___ \| | | ||  _||  _| / _ \| '__|/ __|
~   | |___ | |  | |  |  __/| (__ | |_  | |_/ /| |_| || |  | |  |  __/| |   \__ \
~   \____/ |_|  |_|   \___| \___| \__| \____/  \__,_||_|  |_|   \___||_|   |___/                                                                        
~                                                                            
~
================================================================================== */
// These massive headings may appear to be just for show, but they do wonders when quickly
// scrolling through code

// Buffer wrapper struct, requires address to uint16_t [buffer_size]
struct sample_buffer {

    const uint16_t size;            // Size of buffer (number of samples)
    uint16_t in_count = 0;          // Input counter
    uint16_t out_count = 0;         // Output counter
    int16_t *contents;              // Pointer to actual buffer data

};

// Function to fill buffer with specified value value, only additional function call 
// that is actually utilised, also stays far far away from ADC / PWM interrupts 
// because function call overheads
void clear_sample_buffer(sample_buffer* buff,uint16_t val)
{
    for (uint32_t i = 0; i < buff->size; i++)
    {
        buff->contents[i] = val;
    }
}



/* ============================================================================================
~
~
~    _____  _         _             _   _   _               _         _      _            
~   |  __ \| |       | |           | | | | | |             (_)       | |    | |           
~   | |  \/| |  ___  | |__    __ _ | | | | | |  __ _  _ __  _   __ _ | |__  | |  ___  ___ 
~   | | __ | | / _ \ | '_ \  / _` || | | | | | / _` || '__|| | / _` || '_ \ | | / _ \/ __|
~   | |_\ \| || (_) || |_) || (_| || | \ \_/ /| (_| || |   | || (_| || |_) || ||  __/\__ \
~    \____/|_| \___/ |_.__/  \__,_||_|  \___/  \__,_||_|   |_| \__,_||_.__/ |_| \___||___/
~                                                                                      
~                                                                                       
============================================================================================== */

// Used for calculating time taken to complete ADC / process efect callback
uint32_t time_ended;        
uint32_t time_started;      
int32_t time_taken;   

// Global output volume as a ratio over 128
int32_t output_volume = 127;

// Used to reset microcontroller to programming mode
int32_t reboot_to_bootlader = 0;
int32_t temp_val;   // One of the many temporary values that i forgot what it does

uint16_t adc_raw;   // Raw sample coming from adc

uint16_t level = 688;   // final value used to set PWM output
uint slice;             // PWM hardware slice number
uint channel;           // PWM hardware channel number
uint pwm_callbacks;     // Was once used for debugging
                        // number of times PWM callback occured

// Create timer object
struct repeating_timer timer;      

/* ================================================================================
~
~   ______ _     _             _   _             
~   |  _  (_)   | |           | | (_)            
~   | | | |_ ___| |_ ___  _ __| |_ _  ___  _ __  
~   | | | | / __| __/ _ \| '__| __| |/ _ \| '_ \ 
~   | |/ /| \__ \ || (_) | |  | |_| | (_) | | | |
~   |___/ |_|___/\__\___/|_|   \__|_|\___/|_| |_|
~                                                
~   
================================================================================= */


int32_t distortion_enabled = 0;         // Enable distortion
uint16_t distortion_dry_amplitude = 0;  // Amplitude of clean signal /128

// Select map for climbing / falling signal 
uint16_t *selected_distortion_map = distortion_map_down;    
uint16_t previous_sample_raw = 0;   // Used to determine signal falling / climbing

// Only switch mpas if signal increased / deccreased by certain threshold, used to
// remove noise from low output signal with rapid changes
int32_t distortion_mask = 0xffff;   
int32_t distortion_mix = 100;       // Variable set by user, amplitude of distorted signal /128

// Temporary sample values
uint32_t temp_sample;
uint32_t temp_distorted_sample;


/* ================================================================================
~ 
~    _                         ______             
~   | |                        | ___ \            
~   | |     _____      ________| |_/ /_ _ ___ ___ 
~   | |    / _ \ \ /\ / /______|  __/ _` / __/ __|
~   | |___| (_) \ V  V /       | | | (_| \__ \__ \
~   \_____/\___/ \_/\_/        \_|  \__,_|___/___/
~
~ .                                    
================================================================================= */


int32_t lowpass_enabled = 0;                // Enable lowpass
int32_t lowpass_frequency = 10000;          // cut-off frequency in Hz
int16_t lowpass_coeff_a [3] = {1, 0, 0};    // Low-pass a coefficients
int16_t lowpass_coeff_b [3] = {1, 0, 0};    // Low-pass b coefficients

// Only floating point numbers use, stay far away from ADC / PWM interrupts
// Used to calculate low-pass coefficients
double lopass_cutoff_frequency_rads;
double C,D;             
double temp_double;

// Sample buffer for low-pass filter
int16_t lowpass_buffer_data[LOWPASS_BUFFER_SIZE];
sample_buffer lowpass_buffer = {LOWPASS_BUFFER_SIZE,0,0,lowpass_buffer_data};



/* ================================================================================
~
~    _____      _                  
~   |  _  |    | |                 
~   | | | | ___| |_ __ ___   _____ 
~   | | | |/ __| __/ _` \ \ / / _ \
~   \ \_/ / (__| || (_| |\ V /  __/
~    \___/ \___|\__\__,_| \_/ \___|
~                               
~                               
=============================================================================== */
// Ignore this effect entirely, added last second just for fun, doesnt work
int32_t octave_enabled = 0;


/* ================================================================================
~   
~   ______        _               
~   |  _  \      | |              
~   | | | |  ___ | |  __ _  _   _ 
~   | | | | / _ \| | / _` || | | |
~   | |/ / |  __/| || (_| || |_| |
~   |___/   \___||_| \__,_| \__, |
~                            __/ |
~                           |___/ 
~ .
================================================================================= */

#define DELAY_BUFFER_SIZE 47000        // Buffer size of just over 1s worth of samples, up to 1s delay

int32_t delay_sample_count = 22000;    // Delay length (which b / a coefficient is used)
uint16_t delayed_value = 0;            // Actual sample value, used later

int32_t pre_delay_amplitude = 100;      // Amplitude of b_0 coefficient
int32_t delay_amplitude = 50;           // Amplitude of b_d coefficient as a fraction over 128
int32_t IIR_enabled = 0;                // Replaces b_d coefficient with a_d coefficient, enables feedback
int32_t IIR_amplitude = 10;             // Amplitude of a_d coefficient as a fraction over 128 
                                        // Maximum of 104/128 (0.815) allowed
                                        // This is technically -a coefficient, not relevant here tho

// Delay input buffer
int16_t delay_input_buffer_data[DELAY_BUFFER_SIZE];       // Actual input buffer data
sample_buffer delay_input_buffer = {DELAY_BUFFER_SIZE,0,0,delay_input_buffer_data}; // Input buffer wrapper

// Delay output buffer
int16_t delay_output_buffer_data [DELAY_BUFFER_SIZE];     // Actual output buffer data
sample_buffer delay_output_buffer = {DELAY_BUFFER_SIZE, 0, 0, delay_output_buffer_data}; // Output buffer wrapper

int32_t delay_enabled = 0;              // Enable delay effect

// Determines wether to pull from input buffer (FIR) or output buffer (IIR)
sample_buffer* current_sample_buffer_for_delay = &delay_input_buffer;   
sample_buffer* ADC_input_buffer = &delay_input_buffer;  // Points the ADC to the right buffer to output its samples
uint16_t current_delay_amplitude; // Will be set to either delay_amplitude or IIR_amplitude



/* ================================================================================
~
~   ______  _                                  
~   |  ___|| |                                 
~   | |_   | |  __ _  _ __    __ _   ___  _ __ 
~   |  _|  | | / _` || '_ \  / _` | / _ \| '__|
~   | |    | || (_| || | | || (_| ||  __/| |   
~   \_|    |_| \__,_||_| |_| \__, | \___||_|   
~                             __/ |            
~                            |___/             
~  .                        
================================================================================= */
// I say flanger loosely, does both flanger and chorus
// Piggy-backs off of the delay effect, only adjusts the delay length over time

int32_t flanger_enabled = 0;            // Enable flanger / chorus
int32_t flanger_min_delay = 0;          // Minimum delay in samples
int32_t flanger_max_delay = 2000;       // Maximum delay in samples
int32_t flanger_period = 10;            // Unused?

// Every sampling period flanger will increment a counter. When the counter overflows to this
// value, the delay will be incremented / decremented
int32_t flanger_increment_period = 10;  
int32_t flanger_delta = 1;       // The amount the delay will be incremented / decremented by
bool flanger_climbing = true;    // Wether the delay is increasing or decreasing
uint16_t flanger_timer = 0;      // Actual counter as explained




/* ===========================================================================================
~
~   __   __ _                ______                _           
~   \ \ / /| |               |  ___|              | |          
~    \ V / | |_  _ __   __ _ | |_    _   _  _ __  | | __ _   _ 
~    /   \ | __|| '__| / _` ||  _|  | | | || '_ \ | |/ /| | | |
~   / /^\ \| |_ | |   | (_| || |    | |_| || | | ||   < | |_| |
~   \/   \/ \__||_|    \__,_|\_|     \__,_||_| |_||_|\_\ \__, |
~                                                         __/ |
~                                                        |___/ 
~ .
============================================================================================ */

// Unused, repeated function calling was scrapped due to overheads
// Return modulo count both up and down, by certain increment
uint16_t wrap_count(uint16_t count, int32_t increment,uint16_t wrap)
{
    return (count + wrap + increment) % wrap;
}



/* ===============================================================================================
~
~______  _              _                     __  _____                _                 _ 
~|  _  \(_)            | |                   / / /  __ \              | |               | |
~| | | | _  ___  _ __  | |  __ _  _   _     / /  | /  \/  ___   _ __  | |_  _ __   ___  | |
~| | | || |/ __|| '_ \ | | / _` || | | |   / /   | |     / _ \ | '_ \ | __|| '__| / _ \ | |
~| |/ / | |\__ \| |_) || || (_| || |_| |  / /    | \__/\| (_) || | | || |_ | |   | (_) || |
~|___/  |_||___/| .__/ |_| \__,_| \__, | /_/      \____/ \___/ |_| |_| \__||_|    \___/ |_|
~               | |                __/ |                                                   
~               |_|               |___/                                                    
~   .
=============================================================================================== */

//i2c screen software defines
#define NO_VARIABLES_ON_SCREEN 7        // Max number of variables on screen
#define NO_MODIFIABLE_VARIABLES 20      // Number of variables available to modify
#define CURRENT_VARIABLE modifiable_variables[selected_variable]    // Quick access to current selected variable
#define CURRENT_VALUE *(modifiable_variables[selected_variable].variable_address)   // Quick access to its value

int16_t variable_display_offset = 0;   // Used to move variables visible on screen
int16_t selected_variable = 0;          // Variable highlighted / selected to modify

struct modifiable_variable              // Structure to store all parameters of a variable
{
    char name [16];                     // Display name
    int32_t* variable_address;          // Address to actual variable
    uint16_t min;                       // Minimum value
    uint16_t max;                       // Maximum value
    uint16_t step;                      // Step size
    bool read_only;                     // Necessary for certain variables
};

// All variables available to modify
modifiable_variable modifiable_variables [NO_MODIFIABLE_VARIABLES]=
{
    {"Out Volume", &output_volume,0,255,4,false},            // Output volume, 0-128 in range, > 128 is overdrive
    {"Distortion", &distortion_enabled, 0,1,1,false},        // Enable distortion thru distortion map
    {"Dst.Mix",&distortion_mix,0,127,4,false},               // Amplitude of distorted signal /128, value of clean signal is 1-that
    {"Dst.Mask",&distortion_mask,0xff00,0xffff,0xff,false},  // Mask used to determine when to switch rising/falling map
    {"LoPass EN",&lowpass_enabled,0,1,1,false},              // Enable low-pass filter
    {"LoPass f",&lowpass_frequency,1000,22000,500,false},    // Low pass cut-off frequency in Hz
    {"PrDly Amp",&pre_delay_amplitude,0,127,4,false},        // b0 coefficient when delay is enabled, /128
    {"Dly EN",&delay_enabled,0,1,1,false},                   // Enable / disable delay
    {"Dly Len",&delay_sample_count,2000,45000,500,false},    // Delay length in samples, 500 samples is roughly 11ms
    {"Dly Amp",&delay_amplitude,0,127,4,false},              // b_d coeffiecient /128
    {"IIR EN",&IIR_enabled,0,1,1,false},                     // Enable feedback
    {"IIR Amp",&IIR_amplitude,0,104,8,false},                // a_d coefficient /128, max of 104 or 0.81
    {"Flngr EN",&flanger_enabled,0,1,1,false},               // Enable flanger/chorus (variable delay)
    {"Min FDly",&flanger_min_delay,0,3000,50,false},         // Minimum flanger delay
    {"Max FDly",&flanger_max_delay,0,8000,50,false},         // Maximum flanger delay, 50 samples is roughly 1.1ms 
    {"FlngPrd.",&flanger_period,1,500,1,false},              // Amount of sampling periods before counter increment
    {"FlngDelt",&flanger_delta,1,100,1,false},               // Change in delay once counter expires
    {"OCTV EN",&octave_enabled,0,1,1,false},                 // Garbage octave effect, doesnt work, left just for fun
    {"Time",&time_taken,0,0,0,true},                         // Time measured to complete ADC and effect processing function
    {"Update",&reboot_to_bootlader,0,1,1,false}              // Means i dont have to open the box every time i want to reprogram
};


// Display object, uninitialised
pico_ssd1306::SSD1306 display;

// Interrupt function called on button press
void irq_button_callback()
{

    /* ----------------------------------------------------------------
~     ___      _   _              _  _              _ _ _           
~    | _ )_  _| |_| |_ ___ _ _   | || |__ _ _ _  __| | (_)_ _  __ _ 
~    | _ \ || |  _|  _/ _ \ ' \  | __ / _` | ' \/ _` | | | ' \/ _` |
~    |___/\_,_|\__|\__\___/_||_| |_||_\__,_|_||_\__,_|_|_|_||_\__, |
~                                                              |___/ 
   ------------------------------------------------------------------ */

    // Get value of all digital gpio pins, unfortunately all buttons
    // have to share the same interrupt handling function
    uint32_t gpio_values = gpio_get_all() & GPIO_BUTTON_MASK;
    bool centre_pressed = 0; // decided to skip some code if resyncing function

    switch (gpio_values)    // Determine which button was pressed
    {
    // Resync input and output buffers, do all 'do once' calculations
    case BUTTON_CENTRE_PRESSED:         
        
        // As a lot of the code here takes more time than what is allowable
        // by watchdog timer, these will show up a lot
        watchdog_update();            
        centre_pressed = 1;   

        // Disable entire PWM system
        pwm_set_irq_enabled(slice,false);
        irq_set_enabled(PWM_IRQ_WRAP,false);
        pwm_set_enabled(slice,false);

        // Disable ADC timer
        cancel_repeating_timer(&timer);
        
        // Empty buffers
        clear_sample_buffer(&delay_input_buffer,0);
        watchdog_update();
        clear_sample_buffer(&delay_output_buffer,0);
        watchdog_update();
        clear_sample_buffer(&lowpass_buffer,0);
        watchdog_update();

        // Reset buffer positions
        delay_input_buffer.in_count = 0;
        delay_input_buffer.out_count = 0;
        delay_output_buffer.in_count = 0;
        delay_output_buffer.out_count = 0;
        lowpass_buffer.in_count = 0;
        lowpass_buffer.out_count = 0;

        if (flanger_enabled)
        {
            // Quick check so it doesnt just break everything
            if (flanger_max_delay > flanger_min_delay)  
            {
                delay_sample_count = flanger_min_delay;
                flanger_climbing = true;
            }
            else  // turn it back off, simplest fix
            {
                flanger_enabled = false; // admittedly annoying sometimes
            }
        }


        if (lowpass_enabled)
        {
            ADC_input_buffer = &lowpass_buffer;  // Point adc to output to lowpass buffer
            
            // This where the coefficients for the low pass filter are calculated
            // Printed to serial for debug
            watchdog_update();
            lopass_cutoff_frequency_rads = PI_2 * lowpass_frequency;    
            watchdog_update();
            C = tan(lopass_cutoff_frequency_rads * HALF_SAMPLING_PERIOD);
            watchdog_update();
            D = 1 + C * (ROOT_2 + C);
            watchdog_update();
            temp_double = 2*((C*C)-1)/D;
            watchdog_update();
            lowpass_coeff_a[1] = (int16_t)(temp_double*EXPONENT_SIZE);
            watchdog_update();
            printf("a1 : %f = %d\n",temp_double,lowpass_coeff_a[1]);    // debug
            watchdog_update();
            temp_double = (1 + C * (C - ROOT_2))/D;
            watchdog_update();
            lowpass_coeff_a[2] = (int16_t)(temp_double*EXPONENT_SIZE);
            watchdog_update();
            printf("a2 : %f = %d\n",temp_double,lowpass_coeff_a[2]);
            watchdog_update();
            temp_double = (C*C) / D;
            watchdog_update();
            lowpass_coeff_b[2] = lowpass_coeff_b[0] = (int16_t)(temp_double*EXPONENT_SIZE);
            watchdog_update();
            printf("b02 : %f = %d\n",temp_double,lowpass_coeff_b[2]);
            watchdog_update();
            temp_double = 2*temp_double;
            watchdog_update();
            lowpass_coeff_b[1] = (int16_t)(temp_double*EXPONENT_SIZE);
            watchdog_update();
            printf("b1 : %f = %d\n",temp_double,lowpass_coeff_b[1]);
            watchdog_update();
        }
        else
        {
            ADC_input_buffer = &delay_input_buffer; // Point adc output back to delay input buffer
        }

        watchdog_update();  // one final hoorahh
        // Re-enable A to D timer
        add_repeating_timer_us(-22,timer_callback,NULL,&timer);

        // Reset PWM counter
        pwm_set_counter(slice,0);

         // Renable PWM system
        pwm_set_irq_enabled(slice,true);
        irq_set_enabled(PWM_IRQ_WRAP,true);
        pwm_set_enabled(slice,true);

        gpio_acknowledge_irq(BUTTON_CENTRE,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        break;  // End of centre button case

    // Left and right buttons increment / decrement selected variable, holding down centre
    // Button uses 10x step size
    case BUTTON_LEFT_PRESSED:

        if (!CURRENT_VARIABLE.read_only)
            CURRENT_VALUE -= CURRENT_VARIABLE.step; // Decrease selected variable by its step
        gpio_acknowledge_irq(BUTTON_LEFT,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;

    case BUTTON_LEFT_PRESSED + BUTTON_CENTRE_PRESSED:   // Really like how clean this case looks

        if (!CURRENT_VARIABLE.read_only)
            CURRENT_VALUE -= 10*CURRENT_VARIABLE.step; // Decrease selected variable by its step
        gpio_acknowledge_irq(BUTTON_LEFT,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;
    
    case BUTTON_RIGHT_PRESSED:

        printf("Right button pressed\n");
        if (!CURRENT_VARIABLE.read_only)
            CURRENT_VALUE += CURRENT_VARIABLE.step; // Increase selected variable by its step
        gpio_acknowledge_irq(BUTTON_RIGHT,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;

    case BUTTON_RIGHT_PRESSED + BUTTON_CENTRE_PRESSED:

        if (!CURRENT_VARIABLE.read_only)
            CURRENT_VALUE += 10*CURRENT_VARIABLE.step; // Increase selected variable by its step
        gpio_acknowledge_irq(BUTTON_RIGHT,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;


    // Up and down buttons change selected variable
    case BUTTON_UP_PRESSED:
        
        printf("Up button pressed\n");
        
        // Go to previous variable
        selected_variable = (selected_variable + NO_MODIFIABLE_VARIABLES - 1) % NO_MODIFIABLE_VARIABLES;
        
        // Offset displayed variables if selected variable goes off-screen
        if ((selected_variable - variable_display_offset) >= NO_VARIABLES_ON_SCREEN)
        {variable_display_offset = NO_MODIFIABLE_VARIABLES - 1;}
        else if (selected_variable - variable_display_offset < 0)
        {variable_display_offset--;}
        gpio_acknowledge_irq(BUTTON_UP,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        break;

    case BUTTON_DOWN_PRESSED:
        
        printf("Down button pressed\n");
        
        // Go to next variable
        selected_variable =  (selected_variable + 1) % NO_MODIFIABLE_VARIABLES;

        // Offset displayed variables if selected variable goes off-screen
        if ((selected_variable - variable_display_offset) >= NO_VARIABLES_ON_SCREEN)
        {variable_display_offset++;}
        else if (selected_variable - variable_display_offset < 0)
        {variable_display_offset = 0;}

        gpio_acknowledge_irq(BUTTON_DOWN,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt

        break;

    default: // If unknown / multiple button pressed
        
        // Just acknolwedge everything
        gpio_acknowledge_irq(BUTTON_UP,GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(BUTTON_DOWN,GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(BUTTON_LEFT,GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(BUTTON_RIGHT,GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(BUTTON_CENTRE,GPIO_IRQ_EDGE_RISE);
        
        break;
    }

    // The last section of this interrupt does not cause timing issues

    // Ensure selected variable is within range
    if (!CURRENT_VARIABLE.read_only)
    {
        if ( CURRENT_VALUE > CURRENT_VARIABLE.max) {CURRENT_VALUE = CURRENT_VARIABLE.max;}
        else if (CURRENT_VALUE < CURRENT_VARIABLE.min) {CURRENT_VALUE = CURRENT_VARIABLE.min;}
    }

    if (reboot_to_bootlader)    // Reboot to bootloader / programming mode
    {
        display.clear();
        pico_ssd1306::drawText(&display,font_8x8,"Ready to update...",4,32);
        display.sendBuffer();
        irq_set_mask_enabled(0x1F,false);   // Output LED will flash indicating 
                                            // data transfer :D
        reset_usb_boot(1<<OUTPUT_LED,0); // reset to bootloader mode
        while (1)
        {}
    }    

    // Calcualate split for distortion / dry
    distortion_dry_amplitude = 128 - distortion_mix;

    // Toggle buffers and b/a coefficients between FIR / IIR 
    if (IIR_enabled)
    {
        current_sample_buffer_for_delay = &delay_output_buffer;
        current_delay_amplitude = IIR_amplitude;
    }
    else
    {
        current_sample_buffer_for_delay = &delay_input_buffer;
        current_delay_amplitude = delay_amplitude;
    }


    /* --------------------------------------------------------------
~      _   _          _      _         ___  _         _           
~     | | | |_ __  __| |__ _| |_ ___  |   \(_)____ __| |__ _ _  _ 
~     | |_| | '_ \/ _` / _` |  _/ -_) | |) | (_-< '_ \ / _` | || |
~      \___/| .__/\__,_\__,_|\__\___| |___/|_/__/ .__/_\__,_|\_, |
~           |_|                                 |_|          |__/ 
    ---------------------------------------------------------------- */
    // Last second just in case change, did not seem to cause issues either way
    if (!centre_pressed)   // wont update screen for re-sync button
    {
        display.clear();        // Clear buffer
        char temp_string[20];   // Temporary string
        char selected;          // Highlights selected variable
        
        // For each variable currently on screen
        for (int i = 0; i < (NO_VARIABLES_ON_SCREEN + variable_display_offset) && (i + variable_display_offset) < NO_MODIFIABLE_VARIABLES; i++)
        {
            selected = ' '; // Default -> no highlight

            // Highlight if selected
            if (i + variable_display_offset == selected_variable) 
            {
                if (modifiable_variables[i+variable_display_offset].read_only) 
                    selected = 'x';     // for read only
                else 
                    selected = 'o';     // for other variables
            }
            
            // Format <highlight character> <variable name> <value>
            sprintf(temp_string,"%c %s %d",selected,modifiable_variables[i + variable_display_offset].name,*(modifiable_variables[i+variable_display_offset].variable_address));
            
            // Print to display 
            pico_ssd1306::drawText(&display,font_8x8,temp_string,0,i*8);
        }
        display.sendBuffer();   // Send buffer to screen
        }
}





/* =======================================================================
~
~    _____                  _      ___  _       ______ 
~   |_   _|                | |    / _ \| |      |  _  \
~     | | _ __  _ __  _   _| |_  / /_\ \ |_ ___ | | | |
~     | || '_ \| '_ \| | | | __| |  _  | __/ _ \| | | |
~    _| || | | | |_) | |_| | |_  | | | | || (_) | |/ / 
~    \___/_| |_| .__/ \__,_|\__| \_| |_/\__\___/|___/  
~              | |                                     
~              |_|                                     
~ .
======================================================================= */
// Dont ask me why I decided to call it AtoD and not ADC 
// While this was originally going to only do the AtoD conversion, 
// Due to syncing issues it now does both ADC and processes the effects


// Timer callback function, takes timer object as input 
bool timer_callback(struct repeating_timer *t)
{
    time_started = time_us_32();    // For measuring time
    
    previous_sample_raw = adc_raw;  // Remember previous sample for distortion
    adc_raw = adc_read();           // Get input from A -> D 
    temp_sample = adc_raw;          // Temporary value
   

    // Turn on output LED if value in range
    // Originally used for debug, but as a byproduct of the
    // signal oscillating in and out of range, the LED gets brighter
    // the louder the overall signal is giving some cool indication
    if (adc_raw > 3000 || adc_raw < 1000)
    {
        gpio_pull_up(OUTPUT_LED);
        
        // Indicates signal is on the verge of / clipping outside 
        // ADC range
        if (adc_raw > 3900 || adc_raw < 140)
        {gpio_pull_up(CLIP_LED);}
        else
        {gpio_pull_down(CLIP_LED);}

    }
    else
    {
        gpio_pull_down(OUTPUT_LED);
    }

    /* ----------------------------------------------
    ~  ___  _    _           _   _          
    ~ |   \(_)__| |_ ___ _ _| |_(_)___ _ _  
    ~ | |) | (_-<  _/ _ \ '_|  _| / _ \ ' \ 
    ~ |___/|_/__/\__\___/_|  \__|_\___/_||_|
                                       
    ------------------------------------------------ */

    // Massive nested if statements probably arent the best for performance
    // but they worked well enough
    if (distortion_enabled)
    {
        // Check if signal falling / rising, change distortion map accordingly
        // If mask enabled, it will equal ff00, l.s. 8 bits will not be considered when
        // determining if signal is climbing or falling, works shockingly well
        if ((adc_raw & distortion_mask) > (previous_sample_raw & distortion_mask))
        {
            selected_distortion_map = distortion_map_up;
        } 
        else if ((adc_raw & distortion_mask) < (previous_sample_raw& distortion_mask))
        {
            selected_distortion_map = distortion_map_down;
        }

        // Scale clean and distorted signal by set values
        temp_distorted_sample = (selected_distortion_map[temp_sample] * distortion_mix) >> 7;
        temp_sample = (temp_sample * distortion_dry_amplitude) >> 7;
        // Mix together for final sample
        temp_sample += temp_distorted_sample;

    }

    // Convert to signed integer and place in buffer
    // ADC_input_buffer points to either lowpass buffer or delay input buffer
    // dependant on wether the low-pass filter is enabled
    ADC_input_buffer->contents[ADC_input_buffer->in_count] = temp_sample - 2048;
    
    // Very quick and dirty attempt to add octave effect (twice input frequency)
    // added just for fun, unsurprisingly it does not work at all
    // Sounds like an incredibly distorted mess
    if (octave_enabled)
    {
        ADC_input_buffer->contents[ADC_input_buffer->in_count] = abs(ADC_input_buffer->contents[ADC_input_buffer->in_count]);
    }

    // Increment ADC input buffer position, wrap around if buffer end reached
    ADC_input_buffer->in_count = (ADC_input_buffer->in_count +1) % ADC_input_buffer->size;


    /* --------------------------------------
    ~   _                   ___            
    ~  | |   _____ __ _____| _ \__ _ ______
    ~  | |__/ _ \ V  V /___|  _/ _` (_-<_-<
    ~  |____\___/\_/\_/    |_| \__,_/__/__/
                                    
    -------------------------------------- */

    if (lowpass_enabled)  // sometimes verbose variable names are really useful :D
    {
            // Temporaray samples mid calculation
            uint32_t temp_lopass_sample = 0;
            uint32_t temp_working_sample = 0;
            
            // Go through b coefficients
            for (uint8_t i = 0; i < 3; i++)
            {
                // Take x[n-i], multiply by b_i, add to working sample
                temp_working_sample = 
                (
                    lowpass_buffer.contents[
                    (lowpass_buffer.out_count - i + lowpass_buffer.size) % lowpass_buffer.size ]
                    * lowpass_coeff_b[i]
                ) >> 14;
                temp_lopass_sample += temp_working_sample;

            }
            // Go through a coefficients, a_0 is ignored
            for (uint8_t i = 1; i < 3; i++)
            {
                // Take y[n-i] multiply by a_i
                temp_working_sample = 
                (
                    delay_input_buffer.contents[
                    (delay_input_buffer.in_count - i + delay_input_buffer.size) % delay_input_buffer.size ]
                    * lowpass_coeff_b[i]
                ) >> 14;
                // Samples now subracted to get the same effect as adding -a_iy[n-i]
                temp_lopass_sample -= temp_working_sample;
            }

            // Place sample into correct location in correct buffer, increment counter
            delay_input_buffer.contents[delay_input_buffer.in_count] = temp_lopass_sample;
            lowpass_buffer.out_count = (lowpass_buffer.out_count + 1) % lowpass_buffer.size;
            delay_input_buffer.in_count = (delay_input_buffer.in_count + 1) % delay_input_buffer.size;
            

    }

    /* ------------------------------------------------------------
     ~  ___      _               __  ___ _                        
     ~ |   \ ___| |__ _ _  _    / / | __| |__ _ _ _  __ _ ___ _ _ 
     ~ | |) / -_) / _` | || |  / /  | _|| / _` | ' \/ _` / -_) '_|
     ~ |___/\___|_\__,_|\_, | /_/   |_| |_\__,_|_||_\__, \___|_|  
     ~                  |__/                        |___/         
     ----------------------------------------------------------- */

    // Flanger chours effects pigguback on the delay effect;
    // both need to be enabled
    // Im sure i couldve reduced the nesting but this took enough time
    // as is
    if (flanger_enabled)
    {
        // Inclement flanger timer / counter, wrap to 0 on overflow
        flanger_timer = (flanger_timer + 1) % flanger_period;
        if (flanger_timer == 0) // On overflow
        {
            if (flanger_climbing) 
            {
                // Add specified delay change
                delay_sample_count += flanger_delta; 
                if (delay_sample_count >= flanger_max_delay)
                {
                    // Negate climbing to falling
                    flanger_climbing = !flanger_climbing;
                }
            }
            else    // flanger descending
            {
                // Same but in reverse
                delay_sample_count -= flanger_delta;
                if (delay_sample_count <= flanger_min_delay)
                {
                    // Negate falling to climbing
                    flanger_climbing = !flanger_climbing;
                }
            }
         }
     }

    if (delay_enabled)
    {   
        // Get delayed value (n-d), find x/y [n-d], multiply by correct coefficient
        delayed_value = (delay_input_buffer.out_count - delay_sample_count + delay_input_buffer.size ) % delay_input_buffer.size;
        temp_val = (delay_input_buffer.contents[delay_input_buffer.out_count] * pre_delay_amplitude) >> 7;
        temp_val += (current_sample_buffer_for_delay->contents[delayed_value] *  current_delay_amplitude) >> 7;
        // mix in o.g. signal
    }
    else    // Delay input / output buffers always used, regardless if delay enabled
    {
        temp_val = delay_input_buffer.contents[delay_input_buffer.out_count];
    }

    // Delay output buffer is also the fian buffer the PWM reads from
    delay_output_buffer.contents[delay_output_buffer.in_count] = temp_val ;
    delay_input_buffer.out_count = (delay_input_buffer.out_count + 1) % delay_input_buffer.size;
    
    // if a buffer feeding the next buffer is the same size, the output of the 
    // prior should always be in sync with the input of the latter
    delay_output_buffer.in_count = delay_input_buffer.in_count;

    // Get time taken to complete entire function
    time_ended  = time_us_32();
    time_taken = time_ended - time_started;
    return true;    // Genuinely do not rember if or why this is necessary
                    // Something to do with the timer repeating
}


/* =======================================================================
~
~    _____       _               _    ______ _    ____  ___
~   |  _  |     | |             | |   | ___ \ |  | |  \/  |
~   | | | |_   _| |_ _ __  _   _| |_  | |_/ / |  | | .  . |
~   | | | | | | | __| '_ \| | | | __| |  __/| |/\| | |\/| |
~   \ \_/ / |_| | |_| |_) | |_| | |_  | |   \  /\  / |  | |
~    \___/ \__,_|\__| .__/ \__,_|\__| \_|    \/  \/\_|  |_/
~                   | |                                    
~                   |_|                                    
~ .
======================================================================= */

// Set to (1375 - 1) as the counter needs 1 PWM clock period to roll over to 0
#define PWM_PERIOD 1374 

void pwm_interrupt_callback()
{
    // The level of the PWM is set to the value calculated on the previous sampling period
    // This is so the PWM value is changed ASAP after PWM counter rolls over, but results in
    // a 1 sample (22us) latency
    pwm_set_gpio_level(PWM_PIN,level); 
    pwm_clear_irq(slice);   // Clear interrupt
    if (delay_output_buffer.out_count != delay_output_buffer.in_count)  // Output remains in sync iwht input
    {
        // This heap of code takes the samplie value (-2048 - +2048), scales it by the output volume
        // adds necessary offset to make it (0 - +4096), bitshifts it again to achieve 0 - + 1024 and adds
        // half the difference between 1024 and 1375
        level = (
                    (
                        ((delay_output_buffer.contents[delay_output_buffer.out_count] * output_volume) >> 7) 
                        + 2048
                    )
                    >> 2) 
                    + 175;

        // The output counter of the output buffer is incremented only 
        delay_output_buffer.out_count = (delay_output_buffer.out_count + 1) % delay_output_buffer.size;
    }
    return;
}



/* =======================================================================
~
~   ___  ___      _         _____      _ _   
~   |  \/  |     (_)       |_   _|    (_) |  
~   | .  . | __ _ _ _ __     | | _ __  _| |_ 
~   | |\/| |/ _` | | '_ \    | || '_ \| | __|
~   | |  | | (_| | | | | |  _| || | | | | |_ 
~   \_|  |_/\__,_|_|_| |_|  \___/_| |_|_|\__|
~                                        
~                                         
======================================================================= */

int main() {    // Only really does initialisation and later resets watchdog timer

    /* --------------------------------------
~      ___ _        ___      _             
~     | _ (_)_ _   / __| ___| |_ _  _ _ __ 
~     |  _/ | ' \  \__ \/ -_)  _| || | '_ \
~     |_| |_|_||_| |___/\___|\__|\_,_| .__/
~                                |_|   
    ---------------------------------------- */

    // Initialise LED
    stdio_init_all();
    gpio_init(ONBOARD_LED);

    // Set ADC pin to input
    gpio_set_dir(ADC_PIN, GPIO_IN);

    // Set buttons to inputs
    gpio_set_dir(BUTTON_UP,GPIO_IN);
    gpio_set_dir(BUTTON_DOWN,GPIO_IN);
    gpio_set_dir(BUTTON_LEFT,GPIO_IN);
    gpio_set_dir(BUTTON_RIGHT,GPIO_IN);
    gpio_set_dir(BUTTON_CENTRE,GPIO_IN);

    // Set LED pins to outputs
    gpio_set_dir(CLIP_LED,GPIO_OUT);
    gpio_set_dir(OUTPUT_LED,GPIO_OUT);
    gpio_set_dir(PLAY_LED,GPIO_OUT);
    gpio_set_dir(ONBOARD_LED, GPIO_OUT);

    // Set PWM output pin to PWM mode
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);


    /* ---------------------------------------------
~      ___  _         _             ___      _ _   
~     |   \(_)____ __| |__ _ _  _  |_ _|_ _ (_) |_ 
~     | |) | (_-< '_ \ / _` | || |  | || ' \| |  _|
~     |___/|_/__/ .__/_\__,_|\_, | |___|_||_|_|\__|
~               |_|          |__/                  
    ---------------------------------------------- */

    //Use i2c port with baud rate of 1Mhz
    i2c_init(I2C_PORT, 1000000); 
    //Set pins for I2C operation
    gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_PIN_SDA);
    gpio_pull_up(I2C_PIN_SCL);

    // Instanciate display object
    display = pico_ssd1306::SSD1306(I2C_PORT, I2C_SCREEN_ADDRESS, pico_ssd1306::Size::W128xH64);
    sleep_ms(500);
    display.clear();

    
    /* -----------------------------------------
~      _  _     _ _      __      __       _    _ 
~     | || |___| | |___  \ \    / /__ _ _| |__| |
~     | __ / -_) | / _ \  \ \/\/ / _ \ '_| / _` |
~     |_||_\___|_|_\___/   \_/\_/\___/_| |_\__,_|
                                              
    -------------------------------------------- */

    // 
    if (watchdog_caused_reboot())
    {
        if (watchdog_enable_caused_reboot())
        {
        // Authors note: 
        // My little screen is blue, and inverting the colour of display
        // makes the text black on a blue background
        // Yes, i essentially added a blue-screen and yes
        // Its purpose is exactly what one might imagine
        display.invertDisplay();
        display.clear();
        pico_ssd1306::drawText(&display,font_12x16,"Uh oh..",0,0);
        pico_ssd1306::drawText(&display,font_8x8,"oopsie happen",0,16);
        pico_ssd1306::drawText(&display,font_8x8,"centre to cont.",0,32);
        display.sendBuffer();
        // While this might be silly a short splash screen that indicated
        // microcontroller hanging was incredibly useful when debugging

        // Quick and dirty check for centre button being pressed
        uint32_t gpio_values = 0;
        while (gpio_values != BUTTON_CENTRE_PRESSED)
        {
            gpio_values = gpio_get_all() & GPIO_BUTTON_MASK;
        }
        display.invertDisplay();
        display.clear();
        }
        else    // The pico also uses the watchdog_caused_reboot flag
                // to indicate the microcontroller rebooted to progamming mode
        {
            display.clear();
            pico_ssd1306::drawText(&display,font_12x16,"Update",0,0);
            pico_ssd1306::drawText(&display,font_8x8,"was successful!",0,16);
            display.sendBuffer();
            sleep_ms(1000);
            display.clear();
        }

    }

    // Print setup to screen
    pico_ssd1306::drawText(&display,font_12x16,"FX Pedal",0,0);
    pico_ssd1306::drawText(&display,font_8x8,PROGRAM_VERSION,0,16);
    pico_ssd1306::drawText(&display,font_8x8,"Starting...",0,32);
    display.sendBuffer();
    
    // Turn on onboard led :)
    gpio_put(ONBOARD_LED,1);
    
    // Have a break
    sleep_ms(1000);

    // Print debug info to serial
    printf("--------------------------------------\n");
    printf(PROGRAM_VERSION);
    printf("\n");
    printf("Phase correct pwm \n");
    printf("PWM SLICE: %d\n",slice);
    printf("PWM CHANNEL: %d\n",channel);

    // Have a kit-kat
    sleep_ms(1000);
    gpio_put(ONBOARD_LED,0); // Cant even see it in the box :(

    /* -------------------------------------
~        _  _       ___    ___      _ _   
~       /_\| |_ ___|   \  |_ _|_ _ (_) |_ 
~      / _ \  _/ _ \ |) |  | || ' \| |  _|
~     /_/ \_\__\___/___/  |___|_||_|_|\__|      

    --------------------------------------- */

    // Initialise A->D, assign to pin ADC_PIN
    adc_init();
    adc_gpio_init( ADC_PIN);
    adc_select_input( 0);

    /* -----------------------------------------
~      _____      ____  __   ___      _ _   
~     | _ \ \    / /  \/  | |_ _|_ _ (_) |_ 
~     |  _/\ \/\/ /| |\/| |  | || ' \| |  _|
~     |_|   \_/\_/ |_|  |_| |___|_||_|_|\__|
                                        
    -------------------------------------------*/


    // Get slice and channel number from PWM pin
    slice=pwm_gpio_to_slice_num(PWM_PIN);
    channel =pwm_gpio_to_channel (PWM_PIN);

    // Setup PWM for audio output
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, PWM_PERIOD); 
    pwm_config_set_phase_correct(&config,true);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(PWM_PIN, 1375);

    // Start timer, every 22us, call timer_callback, pass it struct of addresses,
    // use timer object
    pwm_clear_irq(slice);   
    gpio_pull_up(PLAY_LED);

    // ADC timer, -22 indicates that the timer will be reset immediately
    // and not after completion of the function call
    add_repeating_timer_us(-22,timer_callback,NULL,&timer);  

    // Enable PWM and PWM interrupt
    pwm_set_irq_enabled(slice,true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP,pwm_interrupt_callback);
    irq_set_enabled(PWM_IRQ_WRAP,true);
    pwm_set_enabled(slice,true);

    // Force button interrupt to update screen
    irq_button_callback();

    // Button interrupts
    gpio_set_irq_enabled_with_callback(BUTTON_CENTRE, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_UP, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_DOWN, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_RIGHT, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_LEFT, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    
    // Watchdog will reboot the microcontroller if it freezes and display error screen
    watchdog_enable(500,1); // 500ms
    

/* -----------------------------------------------------------------
~
~   ___  ___      _         _                       
~   |  \/  |     (_)       | |                      
~   | .  . | __ _ _ _ __   | |     ___   ___  _ __  
~   | |\/| |/ _` | | '_ \  | |    / _ \ / _ \| '_ \ 
~   | |  | | (_| | | | | | | |___| (_) | (_) | |_) |
~   \_|  |_/\__,_|_|_| |_| \_____/\___/ \___/| .__/ 
~                                            | |    
~                                            |_|    
~ .
-------------------------------------------------------------------- */

    // Loop
    while (true)
    {

        // whole load of nothing here;
        watchdog_update();

        // No way to break from the main loop, irrelevant as the microcontroller
        // Would hang anyways. Only way to restart the microcontroller is forcing
        // a watchdog timeout which I really do not like as I use it to indicate hanging
    }
    
    // Main function does not return 0, wow great code
}
