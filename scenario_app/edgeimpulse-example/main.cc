#include "hx_drv_tflm.h"
#include "tflite-model/trained_model_compiled.h"
#include "uFire_SHT20.h"
#include "hx_example_utils.h"
uFire_SHT20 mySensor;

namespace
{
    hx_drv_sensor_image_config_t g_pimg_config;
    hx_drv_gpio_config_t gpio_config;
    int input_width = 96;
    int input_height = 96;
    int input_channels = 1;
    uint32_t humidity_threshold = 75; // % relative humidity
    uint32_t distance_threshold = 7; // cm
    float intervals = 0.1; // minute
}

TfLiteStatus GetImage(int image_width, int image_height, int channels, int8_t *image_data)
{
    static bool is_initialized = false;

    if (!is_initialized)
    {
        if (hx_drv_sensor_initial(&g_pimg_config) != HX_DRV_LIB_PASS)
        {
            return kTfLiteError;
        }

        if (hx_drv_spim_init() != HX_DRV_LIB_PASS)
        {
            return kTfLiteError;
        }

        is_initialized = true;
    }

    //capture image by sensor
    hx_drv_sensor_capture(&g_pimg_config);

    //send jpeg image data out through SPI
    hx_drv_spim_send(g_pimg_config.jpeg_address, g_pimg_config.jpeg_size,
                     SPI_TYPE_JPG);

    hx_drv_image_rescale((uint8_t *)g_pimg_config.raw_address,
                         g_pimg_config.img_width, g_pimg_config.img_height,
                         image_data, image_width, image_height);

    return kTfLiteOk;
}

unsigned long pingMotor() 
{
  uint32_t i = 0, n = 25;
  while(i<=n)
  {
    gpio_config.gpio_pin = HX_DRV_PGPIO_2;
    gpio_config.gpio_data = 1;
    hx_drv_gpio_set(&gpio_config);
    hx_util_delay_ms(2);

    gpio_config.gpio_pin = HX_DRV_PGPIO_2;
    gpio_config.gpio_data = 0;
    hx_drv_gpio_set(&gpio_config);
    hx_util_delay_ms(18);
    hx_drv_uart_print("motor %d\n",i);      
    i +=1;
  }
  return 0;

}

unsigned long ping() {
    uint32_t tick_start = 0, tick_end = 0;
    gpio_config.gpio_pin = HX_DRV_PGPIO_0;
    gpio_config.gpio_data = 1;
    hx_drv_gpio_set(&gpio_config);
    hx_util_delay_ms(5);

    gpio_config.gpio_pin = HX_DRV_PGPIO_0;
    gpio_config.gpio_data = 0;
    hx_drv_gpio_set(&gpio_config);

    uint32_t time = 0;
    uint32_t i = 0;

    while(1)
    {
        gpio_config.gpio_pin = HX_DRV_PGPIO_1;
        hx_drv_gpio_get(&gpio_config);
        time = gpio_config.gpio_data;
        //hx_drv_uart_print(":%d",time);
        
        //hx_drv_uart_print("%d\n",i);
        if(time == 1 && i == 0)
            {
                hx_drv_tick_start();
                hx_drv_tick_get(&tick_start);
            }
        if (time == 1)
            {
                i++;
            }
        if(time == 0 && i > 0)
            {
                hx_drv_tick_get(&tick_end);
                break;
            }
    }
    uint32_t cm = ((tick_end-tick_start)/4)*0.0001657;
    //hx_drv_uart_print("%d",cm);
    return  (cm) ;  // 換算成 cm 並傳回
  }


unsigned long RespondToDetection(int8_t *score)
{
    // get the index with the highest score
    int maxindex = 0;
    int maxvalue = -128;
    for (int i = 0; i < 3; i++)
    {
        if (score[i] > maxvalue)
        {
            maxvalue = score[i];
            maxindex = i;
        }
    }
    hx_drv_uart_print("[pet] %d, [Human] %d [None] %d \n", score[0], score[1], score[2]);

    uint32_t cat;
    if (maxindex == 0) {
        hx_drv_led_on(HX_DRV_LED_GREEN);
        hx_drv_led_off(HX_DRV_LED_RED);
        cat = 0;
    } else {
        hx_drv_led_on(HX_DRV_LED_RED);
        hx_drv_led_off(HX_DRV_LED_GREEN);
        cat = 1;
    }
      return cat;
    // show the scores to UART
}

unsigned long getCatImage() 
{
    TfLiteTensor *input = trained_model_input(0);
    if (kTfLiteOk != GetImage(input_width, input_height, input_channels, input->data.int8))
    {
        hx_drv_uart_print("Image capture failed.");
    }
    trained_model_invoke();

    TfLiteTensor *output = trained_model_output(0);
    uint32_t cat = RespondToDetection((int8_t *)output->data.uint8);
      return cat;
}

int main(void)
{
    //Sensor
    hx_drv_share_switch(SHARE_MODE_I2CM);
    hx_drv_uart_initial(UART_BR_115200);
    //hx_drv_led_on(HX_DRV_LED_GREEN);
    //hx_drv_led_on(HX_DRV_LED_RED);

    gpio_config.gpio_pin = HX_DRV_PGPIO_0;
    gpio_config.gpio_direction = HX_DRV_GPIO_OUTPUT;
    hx_drv_gpio_initial(&gpio_config);
    gpio_config.gpio_pin = HX_DRV_PGPIO_1;
    gpio_config.gpio_direction = HX_DRV_GPIO_INPUT;
    hx_drv_gpio_initial(&gpio_config);
    gpio_config.gpio_pin = HX_DRV_PGPIO_2;
    gpio_config.gpio_direction = HX_DRV_GPIO_OUTPUT;
    hx_drv_gpio_initial(&gpio_config);

    hx_drv_uart_print("SHT20\n"); 
    hx_drv_uart_print("US100\n");      
    // Init SHT20 Sensor    
    mySensor.begin();    
                        
    if (mySensor.begin() == false)
    {
        hx_drv_uart_print("SHT20 error. Please check wiring. Freezing...\n");
    }
    hx_util_delay_ms(100);

    // init model
    TfLiteStatus init_status = trained_model_init(NULL);
    if (init_status != kTfLiteOk)
    {
        hx_drv_uart_print("init fail\n");
        return 0;
    }


    // loop step
    while (true)
    {
        hx_drv_uart_print("Humidity: %d %%,  Temperature: %d C\n",(uint32_t)mySensor.humidity(),(uint32_t)mySensor.temperature());
        uint32_t humidity = mySensor.humidity();
        if(humidity>humidity_threshold){
            while(true){
                hx_drv_led_on(HX_DRV_LED_RED);
                hx_util_delay_ms(400);
                hx_drv_led_off(HX_DRV_LED_RED);
                hx_util_delay_ms(300);
                uint32_t humidity = mySensor.humidity();
                if (humidity<humidity_threshold){
                    break;
                }
            }
        }
        uint32_t distance  = ping();  
        hx_drv_uart_print("Distance: %d cm\n", distance);
        if(distance>distance_threshold){
            while(true){
                hx_drv_led_on(HX_DRV_LED_GREEN);               
                hx_drv_led_on(HX_DRV_LED_RED);
                uint32_t distance  = ping();  
                if (distance<distance_threshold){
                    hx_drv_led_off(HX_DRV_LED_GREEN);               
                    hx_drv_led_off(HX_DRV_LED_RED);
                    break;
                }
            }
        }
        //hx_util_delay_ms(200);

        uint32_t catNumber = 0;
        uint32_t j = 0;
        uint32_t m = 25;
        while (j <= m) {
            uint32_t isCat = getCatImage(); 
            hx_drv_uart_print("is cat ? :%d \n", isCat);
            if(isCat == 0){
                catNumber += 1;
            } 
            if(catNumber == 15){
                pingMotor();
                hx_drv_led_off(HX_DRV_LED_GREEN);
                hx_util_delay_ms(intervals*60*1000);
                break;
            }
            j++;
        }
    }

    return 0;
}
