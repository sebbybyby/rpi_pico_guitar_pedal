/**
 *  Written by sebus
 *
 * 
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "pico-ssd1306/textRenderer/8x8_font.h"

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
#define OUTPUT_LED 3
#define PLAY_LED 4
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

#define PROGRAM_VERSION "v30_04_22"




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
    for (uint32_t i = 0; i <buff->size; i++)
    {
        buff->contents[i] = val;
    }
}

#define DELAY_BUFFER_SIZE 47000     

// Delay input buffer
int16_t input_buffer_data[DELAY_BUFFER_SIZE];       // Actual input buffer data
sample_buffer input_buffer = {DELAY_BUFFER_SIZE,0,0,input_buffer_data}; // Input buffer wrapper

// Delay output buffer
int16_t output_buffer_data [DELAY_BUFFER_SIZE];     // Actual output buffer data
sample_buffer output_buffer = {DELAY_BUFFER_SIZE, 0, 0, output_buffer_data}; // Output buffer wrapper





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

uint16_t delay_sample_count = 22000;
uint16_t delayed_value = 0;
int32_t temp_val;

uint16_t delay_repeat = 1;
uint16_t address = 0;

uint16_t level = 688;
uint slice;
uint channel;
uint pwm_callbacks;





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
#define NO_MODIFIABLE_VARIABLES 10      // Number of variables available to modify
#define CURRENT_VARIABLE modifiable_variables[selected_variable]    // Quick access to current selected variable
#define CURRENT_VALUE *(modifiable_variables[selected_variable].variable_address)   // Quick access to its value

uint16_t variable_display_offset = 0;   // Used to move variables visible on screen
int16_t selected_variable = 0;          // Variable highlighted / selected to modify

struct modifiable_variable              // Structure to store all parameters of a variable
{
    char name [16];                     // Display name
    uint16_t* variable_address;         // Address to actual variable
    uint16_t min;                       // Minimum value
    uint16_t max;                       // Maximum value
    uint16_t step;                      // Step size
};

// All variables available to modify
modifiable_variable modifiable_variables [NO_MODIFIABLE_VARIABLES]=
{
    {"Length",&delay_sample_count,2000,45000,1000},
    {"Repeat",&delay_repeat,0,10,1},
    {"Sepeat",&delay_repeat,0,10,1},
    {"Tepeat",&delay_repeat,0,10,1},
    {"Uepeat",&delay_repeat,0,10,1},
    {"Vepeat",&delay_repeat,0,10,1},
    {"Wepeat",&delay_repeat,0,10,1},
    {"Var",(uint16_t*)&selected_variable,0,10,1},
    {"addr",&address,0,64000,1},
    {"crash",(uint16_t*)address,0,64000,1}
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
    uint32_t gpio_values = gpio_get_all() & 0x3E0;

    switch (gpio_values)    // Determine which button was pressed
    {

    case BUTTON_CENTRE_PRESSED:

        printf("Centre button pressed\n");
        gpio_acknowledge_irq(BUTTON_CENTRE,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;

    case BUTTON_LEFT_PRESSED:

        printf("Left button pressed\n");
        CURRENT_VALUE -= CURRENT_VARIABLE.step; // Decrease selected variable by its step
        gpio_acknowledge_irq(BUTTON_LEFT,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;
    
    case BUTTON_RIGHT_PRESSED:

        printf("Right button pressed\n");
        CURRENT_VALUE += CURRENT_VARIABLE.step; // Increase selected variable by its step
        gpio_acknowledge_irq(BUTTON_RIGHT,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;

    case BUTTON_UP_PRESSED:
        
        printf("Up button pressed\n");
        
        // Go to previous variable
        selected_variable = (selected_variable + NO_MODIFIABLE_VARIABLES - 1) % NO_MODIFIABLE_VARIABLES;
        
        // Offset displayed variables if selected variable goes off-screen
        while ((selected_variable - variable_display_offset) >= NO_VARIABLES_ON_SCREEN)
        {variable_display_offset++;}
        while (selected_variable - variable_display_offset < 0)
        {variable_display_offset++;}

        gpio_acknowledge_irq(BUTTON_UP,GPIO_IRQ_EDGE_RISE); // Acknowledge interrupt
        
        break;

    case BUTTON_DOWN_PRESSED:
        
        printf("Down button pressed\n");
        
        // Go to next variable
        selected_variable =  (selected_variable + 1) % NO_MODIFIABLE_VARIABLES;

        // Offset displayed variables if selected variable goes off-screen
        while ((selected_variable - variable_display_offset) >= NO_VARIABLES_ON_SCREEN)
        {variable_display_offset++;}
        while (selected_variable - variable_display_offset < 0)
        {variable_display_offset++;}

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
    if ( CURRENT_VALUE > CURRENT_VARIABLE.max) {CURRENT_VALUE = CURRENT_VARIABLE.max;}
    else if (CURRENT_VALUE < CURRENT_VARIABLE.min) {CURRENT_VALUE = CURRENT_VARIABLE.min;}

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
        if (i + variable_display_offset == selected_variable) {selected = 'o';}
        
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
    
    // Get input from A -> D
    uint16_t adc_raw;
    adc_raw = adc_read(); 

    // Convert to signed integer and place in buffer
    input_buffer.contents[input_buffer.in_count] = adc_raw - 2048;
    // Increment buffer position
    input_buffer.in_count = (input_buffer.in_count +1) % input_buffer.size;
    
    // Turn on output LED if value in range
    if (adc_raw > 3000 || adc_raw < 1000)
    {
        gpio_pull_up(OUTPUT_LED);
    }
    else
    {
        gpio_pull_down(OUTPUT_LED);
    }

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

void pwm_interrupt_callback()
{

    level = ((output_buffer.contents[output_buffer.out_count] + 2048) >> 2) + 175;
    pwm_set_gpio_level(PWM_PIN,level);
    output_buffer.out_count++;
    pwm_clear_irq(slice);

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
    sleep_ms(1000);
    display.clear();

    
    /* -----------------------------------------
~      _  _     _ _      __      __       _    _ 
~     | || |___| | |___  \ \    / /__ _ _| |__| |
~     | __ / -_) | / _ \  \ \/\/ / _ \ '_| / _` |
~     |_||_\___|_|_\___/   \_/\_/\___/_| |_\__,_|
                                              
    -------------------------------------------- */

    // Print setup to screen
    pico_ssd1306::drawText(&display,font_12x16,"FX Pedal",0,0);
    pico_ssd1306::drawText(&display,font_8x8,PROGRAM_VERSION,0,16);
    pico_ssd1306::drawText(&display,font_8x8,"Starting...",0,32);
    display.sendBuffer();
    
    // Turn on onboard led
    gpio_put(ONBOARD_LED,1);
    
    // Have a break
    sleep_ms(5000);

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

    // Create timer object
    struct repeating_timer timer;      

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
    pwm_config_set_wrap(&config, 1375); 
    pwm_config_set_phase_correct(&config,true);
    pwm_init(slice, &config, true);
    pwm_set_enabled(slice,true);
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

    // Force button interrupt to update screen
    irq_button_callback();

    // Button interrupts
    gpio_set_irq_enabled_with_callback(BUTTON_CENTRE, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_UP, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_DOWN, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_RIGHT, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_LEFT, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);


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

        if (input_buffer.in_count != input_buffer.out_count)    // So in theory this should keep the input and
                                                                // output buffers in sync, apparently it doesnt
        {
            delayed_value = (input_buffer.out_count - delay_sample_count + input_buffer.size ) % input_buffer.size;
            temp_val = input_buffer.contents[input_buffer.out_count] * 0.5;
            temp_val += input_buffer.contents[delayed_value] * 0.5;
            output_buffer.contents[output_buffer.in_count] = temp_val ;
            input_buffer.out_count = (input_buffer.out_count + 1) % input_buffer.size;
            output_buffer.in_count = input_buffer.in_count;
        }


    }
}
