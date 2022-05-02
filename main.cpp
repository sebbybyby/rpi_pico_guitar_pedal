/**
 *  Written by sebus
 *
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


#define GPIO_BUTTON_MASK 0x3E0
#define PROGRAM_VERSION "v02_05_22"

// Priority flags for RP2040 interrupts
#define IRQ_LOW_PRIORITY 0xc0
#define IRQ_DEFAULT_PRIORITY 0x80
#define IRQ_HIGH_PRIORITY 0x40
#define IRQ_MAX_PRIORITY 0x00

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


// Buffer wrapper struct, requires address to uint16_t [buffer_size]
struct sample_buffer {

    const uint16_t size;            // Size of buffer (number of samples)
    uint16_t in_count = 0;          // Input counter
    uint16_t out_count = 0;         // Output counter
    int16_t *contents;              // Pointer to actual buffer data

};

// Function to fill buffer with specified value val
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

uint32_t time_ended;
uint32_t time_started;
int32_t time_taken;
int32_t output_volume = 127;


//float calculated_distortion_amplitude = 1;
//float calculated_distortion_dry_amplitude = 0;
//float calculated_pre_delay_amplitude = 1;
//float calculated_delay_amplitude = 0.5;
//float calculated_IIR_amplitude = 0.3;

int32_t reboot_to_bootlader = 0;
int32_t temp_val;

uint16_t adc_raw;

uint16_t level = 688;
uint slice;
uint channel;
uint pwm_callbacks;
uint16_t distortion_dry_amplitude = 0;

uint32_t temp_sample;
uint32_t temp_distorted_sample;



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

int32_t distortion_enabled = 0;
uint16_t *selected_distortion_map = distortion_map_down;
uint16_t previous_sample_raw = 0;
int32_t distortion_mask = 0xffff;
int32_t distortion_mix = 100;
int32_t pre_delay_amplitude = 100;



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

#define DELAY_BUFFER_SIZE 47000     

int32_t delay_sample_count = 22000;
uint16_t delayed_value = 0;


int32_t delay_amplitude = 50;
int32_t IIR_enabled = 0;
int32_t IIR_amplitude = 10;

// Delay input buffer
int16_t input_buffer_data[DELAY_BUFFER_SIZE];       // Actual input buffer data
sample_buffer input_buffer = {DELAY_BUFFER_SIZE,0,0,input_buffer_data}; // Input buffer wrapper

// Delay output buffer
int16_t output_buffer_data [DELAY_BUFFER_SIZE];     // Actual output buffer data
sample_buffer output_buffer = {DELAY_BUFFER_SIZE, 0, 0, output_buffer_data}; // Output buffer wrapper

int32_t delay_enabled = 0;

sample_buffer* current_sample_buffer_to_input_from = &input_buffer;
//float current_delay_amplitude = calculated_delay_amplitude;
uint16_t current_delay_amplitude;



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

int32_t flanger_enabled = 0;
int32_t flanger_min_delay = 0;
int32_t flanger_max_delay = 2000;
int32_t flanger_period = 10;
int32_t flanger_increment_period = 10;

bool flanger_climbing = true;
uint16_t flanger_timer = 0;

/*
repeating_timer flanger_timer;

bool update_flanger_delay(struct repeating_timer *t)
{
    if (flanger_climbing)
    {
        delay_sample_count ++;
    }
    else
    {
        delay_sample_count --;
    }
    if(delay_sample_count <= flanger_min_delay || delay_sample_count >= flanger_max_delay)
    {
        flanger_climbing = !(flanger_climbing);
    }
    return true;
}

*/


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

// Unused
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
#define NO_MODIFIABLE_VARIABLES 16      // Number of variables available to modify
#define CURRENT_VARIABLE modifiable_variables[selected_variable]    // Quick access to current selected variable
#define CURRENT_VALUE *(modifiable_variables[selected_variable].variable_address)   // Quick access to its value

int16_t variable_display_offset = 0;   // Used to move variables visible on screen
int16_t selected_variable = 0;          // Variable highlighted / selected to modify

struct modifiable_variable              // Structure to store all parameters of a variable
{
    char name [16];                     // Display name
    int32_t* variable_address;         // Address to actual variable
    uint16_t min;                       // Minimum value
    uint16_t max;                       // Maximum value
    uint16_t step;                      // Step size
    bool read_only;
};

// All variables available to modify
modifiable_variable modifiable_variables [NO_MODIFIABLE_VARIABLES]=
{
    {"Out Volume", &output_volume,0,255,8,false},
    {"Distortion", &distortion_enabled, 0,1,1,false},
    {"Dst.Mix",&distortion_mix,0,127,8,false},
    {"Dst.Mask",&distortion_mask,0xff00,0xffff,0xff,false},
    {"PrDly Amp",&pre_delay_amplitude,0,127,8,false},
    {"Dly EN",&delay_enabled,0,1,1,false},
    {"Dly Len",&delay_sample_count,2000,45000,1000,false},
    {"Dly Amp",&delay_amplitude,0,127,8,false},
    {"IIR EN",&IIR_enabled,0,1,1,false},
    {"IIR Amp",&IIR_amplitude,0,96,8,false},
    {"Flngr EN",&flanger_enabled,0,1,1,false},
    {"Min FDly",&flanger_min_delay,0,3000,100,false},
    {"Max FDly",&flanger_max_delay,0,8000,100,false},
    {"FlngPrd.",&flanger_period,1,100,1,false},
    //{"FIncPrd",(int32_t*)&flanger_increment_period,0,0,0,true},
    {"Time",&time_taken,0,0,0,true},
    {"Update",&reboot_to_bootlader,0,1,1,false}
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

    // Get value of all digital gpio pins
    uint32_t gpio_values = gpio_get_all() & GPIO_BUTTON_MASK;

    switch (gpio_values)    // Determine which button was pressed
    {

    case BUTTON_CENTRE_PRESSED:         // Resync input and output buffers
        
        // Disable entire PWM system
        pwm_set_irq_enabled(slice,false);
        irq_set_enabled(PWM_IRQ_WRAP,false);
        pwm_set_enabled(slice,false);

        // Disable A to D timer
        cancel_repeating_timer(&timer);
        //cancel_repeating_timer(&flanger_timer);
        
        // Empty buffers
        clear_sample_buffer(&input_buffer,0);
        clear_sample_buffer(&output_buffer,0);

        // Reset buffer positions
        input_buffer.in_count = 0;
        input_buffer.out_count = 0;
        output_buffer.in_count = 0;
        output_buffer.out_count = 0;

        if (flanger_enabled)
        {
            if (flanger_max_delay > flanger_min_delay)
            {
                modifiable_variables[5].read_only = true;
                modifiable_variables[6].read_only = true;
                delay_sample_count = flanger_min_delay;
                flanger_climbing = true;
            }
            else
            {
                modifiable_variables[5].read_only = false;
                modifiable_variables[6].read_only = false;
                flanger_enabled = false;
            }
        }
        else
        {
            modifiable_variables[5].read_only = false;
            modifiable_variables[6].read_only = false;
        }
        
        // Re-enable A to D timer
        add_repeating_timer_us(-22,timer_callback,NULL,&timer);

        // Reset PWM counter
        pwm_set_counter(slice,0);

         // Renable PWM system
        pwm_set_irq_enabled(slice,true);
        irq_set_enabled(PWM_IRQ_WRAP,true);
        pwm_set_enabled(slice,true);

        
        gpio_acknowledge_irq(BUTTON_CENTRE,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;

    case BUTTON_LEFT_PRESSED:

        printf("Left button pressed\n");
        if (!CURRENT_VARIABLE.read_only)
            CURRENT_VALUE -= CURRENT_VARIABLE.step; // Decrease selected variable by its step
        gpio_acknowledge_irq(BUTTON_LEFT,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;
    
    case BUTTON_RIGHT_PRESSED:

        printf("Right button pressed\n");
        if (!CURRENT_VARIABLE.read_only)
            CURRENT_VALUE += CURRENT_VARIABLE.step; // Increase selected variable by its step
        gpio_acknowledge_irq(BUTTON_RIGHT,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;

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

    // Ensure selected variable is within range
    if (!CURRENT_VARIABLE.read_only)
    {
        if ( CURRENT_VALUE > CURRENT_VARIABLE.max) {CURRENT_VALUE = CURRENT_VARIABLE.max;}
        else if (CURRENT_VALUE < CURRENT_VARIABLE.min) {CURRENT_VALUE = CURRENT_VARIABLE.min;}
    }

    if (reboot_to_bootlader) 
    {
        display.clear();
        pico_ssd1306::drawText(&display,font_8x8,"Ready to update...",4,32);
        display.sendBuffer();
        irq_set_mask_enabled(0x1F,false);
        reset_usb_boot(1<<OUTPUT_LED,0); // reset to bootloader mode
        while (1)
        {}
    }    

    distortion_dry_amplitude = 128 - distortion_mix;


    if (IIR_enabled)
    {
        current_sample_buffer_to_input_from = &output_buffer;
        current_delay_amplitude = IIR_amplitude;
    }
    else
    {
        current_sample_buffer_to_input_from = &input_buffer;
        current_delay_amplitude = delay_amplitude;
    }


    /* --------------------------------------------------------------
~      _   _          _      _         ___  _         _           
~     | | | |_ __  __| |__ _| |_ ___  |   \(_)____ __| |__ _ _  _ 
~     | |_| | '_ \/ _` / _` |  _/ -_) | |) | (_-< '_ \ / _` | || |
~      \___/| .__/\__,_\__,_|\__\___| |___/|_/__/ .__/_\__,_|\_, |
~           |_|                                 |_|          |__/ 
    ---------------------------------------------------------------- */

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
                selected = 'x'; 
            else 
                selected = 'o';
        }
        
        // Format <highlight character> <variable name> <value>
        sprintf(temp_string,"%c %s %d",selected,modifiable_variables[i + variable_display_offset].name,*(modifiable_variables[i+variable_display_offset].variable_address));
        
        // Print to display 
        pico_ssd1306::drawText(&display,font_8x8,temp_string,0,i*8);
    }
    display.sendBuffer();   // Send buffer to screen
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



// Timer callback function, takes timer object as input 
bool timer_callback(struct repeating_timer *t)
{
    time_started = time_us_32();
    // Get input from A -> D
    previous_sample_raw = adc_raw;
    adc_raw = adc_read(); 
    temp_sample = adc_raw;
    //uint16_t temp_sample = adc_raw;
    //uint16_t distorted_sample;



    // Turn on output LED if value in range
    if (adc_raw > 3000 || adc_raw < 1000)
    {
        gpio_pull_up(OUTPUT_LED);
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


    if (distortion_enabled)
    {
        if ((adc_raw & distortion_mask) > (previous_sample_raw & distortion_mask))
        {
            selected_distortion_map = distortion_map_up;
        } 
        else if ((adc_raw & distortion_mask) < (previous_sample_raw& distortion_mask))
        {
            selected_distortion_map = distortion_map_down;
        }

        //temp_sample = (temp_sample*calculated_distortion_dry_amplitude)+(selected_distortion_map[temp_sample]*calculated_distortion_amplitude);
        temp_distorted_sample = (selected_distortion_map[temp_sample] * distortion_mix) >> 7;
        temp_sample = (temp_sample * distortion_dry_amplitude) >> 7;
        //tight_loop_contents();
        temp_sample += temp_distorted_sample;


        
        //temp_sample = selected_distortion_map[temp_sample];
    }

    // Convert to signed integer and place in buffer
    input_buffer.contents[input_buffer.in_count] = temp_sample - 2048;
    // Increment buffer position
    input_buffer.in_count = (input_buffer.in_count +1) % input_buffer.size;

    /* ------------------------------------------------------------
     ~  ___      _               __  ___ _                        
     ~ |   \ ___| |__ _ _  _    / / | __| |__ _ _ _  __ _ ___ _ _ 
     ~ | |) / -_) / _` | || |  / /  | _|| / _` | ' \/ _` / -_) '_|
     ~ |___/\___|_\__,_|\_, | /_/   |_| |_\__,_|_||_\__, \___|_|  
     ~                  |__/                        |___/         
     ----------------------------------------------------------- */

    //if (input_buffer.in_count != input_buffer.out_count)    // So in theory this should keep the input and
                                                                // output buffers in sync, apparently it doesnt
    //   {

            if (flanger_enabled)
            {
                flanger_timer = (flanger_timer + 1) % flanger_period;
                if (flanger_timer == 0)
                {
                    if (flanger_climbing) 
                    {
                        delay_sample_count ++; 
                        if (delay_sample_count >= flanger_max_delay)
                        {
                            flanger_climbing = !flanger_climbing;
                        }
                    }
                    else
                    {
                        delay_sample_count --;
                        if (delay_sample_count <= flanger_min_delay)
                        {
                            flanger_climbing = !flanger_climbing;
                        }
                    }
                }

                
            }

            if (delay_enabled)
            {
                delayed_value = (input_buffer.out_count - delay_sample_count + input_buffer.size ) % input_buffer.size;
                temp_val = (input_buffer.contents[input_buffer.out_count] * pre_delay_amplitude) >> 7;
                temp_val += (current_sample_buffer_to_input_from->contents[delayed_value] *  current_delay_amplitude) >> 7;

            }
            else
            {
                temp_val = input_buffer.contents[input_buffer.out_count];
            }

            output_buffer.contents[output_buffer.in_count] = temp_val ;
            input_buffer.out_count = (input_buffer.out_count + 1) % input_buffer.size;
            output_buffer.in_count = input_buffer.in_count;
    //    }

    time_ended  = time_us_32();
    time_taken = time_ended - time_started;
    return true;
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

#define PWM_PERIOD 1374

void pwm_interrupt_callback()
{
    pwm_set_gpio_level(PWM_PIN,level);
    pwm_clear_irq(slice);
    if (output_buffer.out_count != output_buffer.in_count)
    {
        level = (
                    (
                        ((output_buffer.contents[output_buffer.out_count] * output_volume) >> 7) 
                        + 2048
                    )
                    >> 2) 
                    + 175;
        output_buffer.out_count = (output_buffer.out_count + 1) % output_buffer.size;
        //level = (output_volume * level) >> 7;
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

int main() {

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

    if (watchdog_caused_reboot())
    {
        if (watchdog_enable_caused_reboot())
        {
        display.invertDisplay();
        display.clear();
        pico_ssd1306::drawText(&display,font_12x16,"Uh oh..",0,0);
        pico_ssd1306::drawText(&display,font_8x8,"oopsie happend",0,16);
        pico_ssd1306::drawText(&display,font_8x8,"centre to cont.",0,32);
        display.sendBuffer();


        uint32_t gpio_values = 0;
        while (gpio_values != BUTTON_CENTRE_PRESSED)
        {
            gpio_values = gpio_get_all() & GPIO_BUTTON_MASK;
        }
        display.invertDisplay();
        display.clear();
        }
        else
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
    
    // Turn on onboard led
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
    gpio_put(ONBOARD_LED,0);

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




    add_repeating_timer_us(-22,timer_callback,NULL,&timer);  
    sleep_us(10);
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
    watchdog_enable(100,1);
    

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


    }
    
}
