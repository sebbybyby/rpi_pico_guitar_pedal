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
//#include "hardware/sync.h"

#define ADC_PIN 26          // A -> D Pin
#define PWM_PIN 16
#define MAX_LENGTH 44100    // Buffer sample length
#define LED_PIN  25

#define I2C_PIN_SDA 0
#define I2C_PIN_SCL 1
#define I2C_PORT i2c0
#define I2C_SCREEN_ADDRESS 0x3c

#define CLIP_LED 2
#define OUTPUT_LED 3
#define PLAY_LED 4

#define BUTTON_LEFT 5
#define BUTTON_RIGHT 9
#define BUTTON_UP 7
#define BUTTON_DOWN 8
#define BUTTON_CENTRE 6

#define BUTTON_LEFT_PRESSED 0x20
#define BUTTON_RIGHT_PRESSED 0x200
#define BUTTON_UP_PRESSED 0x80
#define BUTTON_DOWN_PRESSED 0x100
#define BUTTON_CENTRE_PRESSED 0x40

#define DELAY_BUFFER_SIZE 47000


uint16_t level = 688;
uint slice;
uint channel;
uint pwm_callbacks;




struct buffer {

    const uint16_t size;
    uint16_t in_count = 0;
    uint16_t out_count = 0;
    uint16_t *contents;

};



uint16_t input_buffer_input_counter = 0;
uint16_t input_buffer_output_counter = 0;
uint16_t input_buffer[46000];
uint16_t input_buffer_size = 46000;


uint16_t output_buffer_data [DELAY_BUFFER_SIZE];
buffer 


//uint16_t output_buffer_input_counter = 0;
//uint16_t output_buffer_output_counter = 0;
//uint16_t output_buffer[46000];
//uint16_t output_buffer_size = 46000;

uint16_t delay_sample_count = 22000;
uint16_t delayed_value = 0;

int32_t temp_val;


pico_ssd1306::SSD1306 display;

struct modifiable_variable 
{
    char name [16];
    uint16_t* variable_address;
    uint16_t min;
    uint16_t max;
    uint16_t step;
};

uint16_t delay_repeat = 1;

#define NO_MODIFIABLE_VARIABLES 2
uint16_t selected_variable = 0;


//pico_ssd1306::SSD1306 display;

#define CURRENT_VARIABLE modifiable_variables[selected_variable]
#define CURRENT_VALUE *(modifiable_variables[selected_variable].variable_address)

modifiable_variable modifiable_variables [NO_MODIFIABLE_VARIABLES]=
{
    {"Length",&delay_sample_count,2000,45000,1000},
    {"Repeat",&delay_repeat,0,10,1}
};



uint16_t wrap_count(uint16_t count, int32_t increment,uint16_t wrap)
{
    return (count + wrap + increment) % wrap;
}

void irq_button_callback()
{

    uint32_t gpio_values = gpio_get_all() & 0x3E0;

    printf("%d\n",gpio_values);

    switch (gpio_values)
    {

    case BUTTON_CENTRE_PRESSED:
        printf("Centre button pressed\n");
        
        gpio_acknowledge_irq(BUTTON_CENTRE,GPIO_IRQ_EDGE_RISE);

        break;

    case BUTTON_LEFT_PRESSED:
        gpio_acknowledge_irq(BUTTON_LEFT,GPIO_IRQ_EDGE_RISE);
        printf("Left button pressed\n");
        *(CURRENT_VARIABLE.variable_address) -= CURRENT_VARIABLE.step;
        break;
    
    case BUTTON_RIGHT_PRESSED:
        gpio_acknowledge_irq(BUTTON_RIGHT,GPIO_IRQ_EDGE_RISE);
        printf("Right button pressed\n");
        *(CURRENT_VARIABLE.variable_address) += CURRENT_VARIABLE.step;
        
    
        break;

    case BUTTON_UP_PRESSED:
        gpio_acknowledge_irq(BUTTON_UP,GPIO_IRQ_EDGE_RISE);
        printf("Up button pressed\n");
        selected_variable = (selected_variable + 1) % NO_MODIFIABLE_VARIABLES;
        break;

    case BUTTON_DOWN_PRESSED:
        gpio_acknowledge_irq(BUTTON_DOWN,GPIO_IRQ_EDGE_RISE);
        printf("Down button pressed\n");
        selected_variable = (selected_variable + NO_MODIFIABLE_VARIABLES - 1) % NO_MODIFIABLE_VARIABLES;
        break;

    default:
        gpio_acknowledge_irq(BUTTON_UP,GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(BUTTON_DOWN,GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(BUTTON_LEFT,GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(BUTTON_RIGHT,GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(BUTTON_CENTRE,GPIO_IRQ_EDGE_RISE);
        break;
    }

    if ( CURRENT_VALUE > CURRENT_VARIABLE.max) {CURRENT_VALUE = CURRENT_VARIABLE.max;}
    else if (CURRENT_VALUE < CURRENT_VARIABLE.min) {CURRENT_VALUE = CURRENT_VARIABLE.min;}

    display.clear();
    char temp_string[20];
    char selected;
    for (int i = 0; i < NO_MODIFIABLE_VARIABLES; i++)
    {
        selected = ' ';
        if (i == selected_variable) {selected = 'o';}
        sprintf(temp_string,"%c %s %d",selected,modifiable_variables[i].name,*(modifiable_variables[i].variable_address));
        pico_ssd1306::drawText(&display,font_8x8,temp_string,0,i*8);
    }
    display.sendBuffer();
}


// Data to pass to timer callback function
struct timer_callback_data
{
    uint* buffer_size;      // Pointer to current sample in buffer
    uint16_t* buffer_ptr;   // Pointer to sample buffer
    uint16_t slice_num;
    uint16_t channel_num;
};



// Timer callback function, takes timer object as input 
bool timer_callback(struct repeating_timer *t)
{
    //struct timer_callback_data *tc_data;    // Define data struct
    //tc_data = (timer_callback_data*)t->user_data;                 // Collect data structure from timer object


    // Exit if buffer is full
    //if (*(tc_data->buffer_size) >= MAX_LENGTH) {return true;}
    
    // Get input from A -> D
    uint16_t adc_raw;
    adc_raw = adc_read(); 

    //printf("i=%d\n",input_buffer_input_counter);
    input_buffer[input_buffer_input_counter] = adc_raw;
    input_buffer_input_counter = (input_buffer_input_counter +1) % input_buffer_size;
    
    //level = (adc_raw >> 2)+175;
    //tc_data->buffer_ptr[0] = level;

    //pwm_set_chan_level(tc_data->slice_num,tc_data->channel_num,level);

    if (adc_raw > 3000 || adc_raw < 1000)
    {
        gpio_pull_up(OUTPUT_LED);
    }
    else
    {
        gpio_pull_down(OUTPUT_LED);
    }
    
    // Store input in buffer[current sample] and move onto next sample
    //tc_data->buffer_ptr[*(tc_data->buffer_size)] = adc_raw;
    //(*(tc_data->buffer_size))++;



    return true;
}

void pwm_interrupt_callback()
{
    
    //printf("%d , %d \n",input_buffer_input_counter,input_buffer_output_counter);
    //if (output_buffer_output_counter == output_buffer_input_counter) 
    //{ 
    //    printf("Not keeping up!\n");
        //return;
    //}
    //else
    //{
    
    pwm_set_gpio_level(PWM_PIN,level);
    delayed_value = (input_buffer_output_counter - delay_sample_count + input_buffer_size ) % input_buffer_size;
    temp_val = (input_buffer[input_buffer_output_counter] - 2048)/2;
    temp_val += (input_buffer[delayed_value] - 2048)/2;
    input_buffer_output_counter = (input_buffer_output_counter + 1) % input_buffer_size;
    level = ((temp_val + 2048) >> 2 ) +175  ;

    pwm_clear_irq(slice);

}

// Main
int main() {

    pwm_callbacks = 0;
    // Initialise LED
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(BUTTON_UP,GPIO_IN);
    gpio_set_dir(BUTTON_DOWN,GPIO_IN);
    gpio_set_dir(BUTTON_LEFT,GPIO_IN);
    gpio_set_dir(BUTTON_RIGHT,GPIO_IN);
    gpio_set_dir(BUTTON_CENTRE,GPIO_IN);

    gpio_set_dir(CLIP_LED,GPIO_OUT);
    gpio_set_dir(OUTPUT_LED,GPIO_OUT);
    gpio_set_dir(PLAY_LED,GPIO_OUT);
    gpio_set_dir(ADC_PIN, GPIO_IN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

    slice=pwm_gpio_to_slice_num(PWM_PIN);
    channel =pwm_gpio_to_channel (PWM_PIN);

    i2c_init(I2C_PORT, 1000000); //Use i2c port with baud rate of 1Mhz
    //Set pins for I2C operation
    gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_PIN_SDA);
    gpio_pull_up(I2C_PIN_SCL);

    display = pico_ssd1306::SSD1306(I2C_PORT, I2C_SCREEN_ADDRESS, pico_ssd1306::Size::W128xH64);
    
    //pwm_set_enabled (slice, true) ;	
    //pwm_set_phase_correct (slice, true);
    //pwm_set_wrap(slice, 1375);
    gpio_put(LED_PIN,1);
    sleep_ms(5000);

    printf("--------------------------------------\n");
    printf("Version 0.22.1404b\n");
    printf("Phase correct pwm \n");
    printf("PWM SLICE: %d\n",slice);
    printf("PWM CHANNEL: %d\n",channel);
    /*
    for (uint16_t i = 0; i < input_buffer_size; i++)
    {
        printf("%d\n",i);
        input_buffer[i] = 2048;
        output_buffer[i] = 2048;
    }
    */
    


    sleep_ms(1000);
    gpio_put(LED_PIN,0);

    

    // Initialise A->D, assign to pin ADC_PIN
    adc_init();
    adc_gpio_init( ADC_PIN);
    adc_select_input( 0);

    struct repeating_timer timer;      // Timer object
    //uint buffer_size = 0;               // Index of current sample
    //uint16_t buffer[MAX_LENGTH];        // Buffer of 16 bit integers for storing samples

    struct timer_callback_data tc_data; // Data struct to pass to timer callback func.
    //tc_data.buffer_size = &buffer_size; // Get address of sample index
    //tc_data.buffer_ptr = buffer;        // Get address of sample buffer
    //tc_data.slice_num = slice;
    //tc_data.channel_num = channel;





    // Setup PWM for audio output
    pwm_config config = pwm_get_default_config();
    /* Base clock 176,000,000 Hz divide by wrap 250 then the clock divider further divides
     * to set the interrupt rate. 
     * 
     * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
     * 
     * 
     * So clkdiv should be as follows for given sample rate
     *  8.0f for 11 KHz
     *  4.0f for 22 KHz
     *  2.0f for 44 KHz etc
     */
    //pwm_config_set_clkdiv(&config, 8.0f); 
    printf("i=%d,o=%d\n",input_buffer_input_counter,input_buffer_output_counter);
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

    irq_button_callback();


    // Button interrupts
    gpio_set_irq_enabled_with_callback(BUTTON_CENTRE, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_UP, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_DOWN, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_RIGHT, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_LEFT, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)&irq_button_callback);


    // Loop
    while (true)
    {
        //printf("i=%d,o=%d\n",input_buffer_input_counter,input_buffer_output_counter);
        //if (input_buffer_input_counter != input_buffer_output_counter)
        //{

        //}
        tight_loop_contents();
        /*
        // Every time buffer fills up
        if (buffer_size >= MAX_LENGTH)
        {

            gpio_put(LED_PIN,0);                // Turn LED off
            cancel_repeating_timer(&timer);     // Temporarily disable timer
            for (uint i = 0; i<MAX_LENGTH; i++) // Flush each sample in buffer
            {                                   // through USB Serial port
                printf("%d,\n",buffer[i]);
            }
            buffer_size = 0;                    // Reset sample index
            // Reenable timer
            add_repeating_timer_us(22,timer_callback,&tc_data,&timer);
            gpio_put(LED_PIN,1);                // Turn LED on
        }
        sleep_ms(500);  // Not strictly necessary
        */
        //tight_loop_contents();
        //sleep_ms(1000);
        //printf("-----------------------");
        //printf("Level: %d\n",level);
        //printf("PWM callbacks: %d\n",pwm_callbacks);

    }
}
