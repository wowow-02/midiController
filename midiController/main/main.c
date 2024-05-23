#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include <string.h>

#define BUF_SIZE (1024)
#define LED_GPIO_PIN GPIO_NUM_2 
#define TXD_PIN GPIO_NUM_17  
#define RXD_PIN GPIO_NUM_16  


uint8_t ble_addr_type;
void ble_app_advertise(void);

//Karakteristika za slanje podaka ESP32-u
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    printf("Received data: %s\n", ctxt->om->om_data);
    char *data = (char *)malloc(ctxt->om->om_len + 1);
    if (data == NULL) {
        

        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    
    memcpy(data, ctxt->om->om_data, ctxt->om->om_len);
    data[ctxt->om->om_len] = '\0';
    int messType,messChnl,param1,param2;
    printf("Data is: %s\n",data);  
    //provjera ispravnosti poruke
    char letterControll[]={'H','R','O','F','P','C','B'};
    int pass=0;

    for(int i=0;i<7;i++){
        if(data[0]==letterControll[i]){
            pass=1;
            break;
        }
    }

    if(ctxt->om->om_len != 9 && ctxt->om->om_len != 6){
        pass=0;
    }
    printf("Pass is %d\n",pass);
    if(pass == 0){
        gpio_set_level(LED_GPIO_PIN,1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO_PIN,0);
        return 0;
    }
    //poruka sa 3 parametra
    if(data[0] != 'H' && data[0]!='R'){
        switch (data[0])
        {
        case 'O':
            messType=0x90;
            break;
        case 'F':
            messType=0x80;
            break;
        case 'P':
            messType=0xA0;
            break;
        case 'C':
            messType=0xB0;
            break;
        case 'B':
            messType=0xE0;
            break;
        default:
            messType=0x90;
            break;
        }
        messChnl=(data[1]-48)*10;
        messChnl=messChnl+(data[2]-48)-1;
        
        param1=(data[3]-48)*100;
        param1=param1+(data[4]-48)*10;
        param1=param1+(data[5]-48);

        param2=(data[6]-48)*100;
        param2=param2+(data[7]-48)*10;
        param2=param2+(data[8]-48);

        messType=messType+messChnl;

        uint8_t midiMess[]={messType,param1,param2};
        int bytesWritten=uart_write_bytes(UART_NUM_2, (const char*)midiMess, sizeof(midiMess));
        if(bytesWritten != sizeof(midiMess)){
            printf("Error writing to UART\n");
        }else{
            printf("Data sent to UART\n");
        }
    }else{
        if(data[0] == 'R'){
            messType=0xC0;
        }else{
            messType=0xD0;
        }
        
        messChnl=(data[1]-48)*10;
        messChnl=messChnl+(data[2]-48)-1;
        messType=messType+messChnl;

        param1=(data[3]-48)*100;
        param1=param1+(data[4]-48)*10;
        param1=param1+(data[5]-48);
        
        uint8_t midiMess[]={messType,param1};
            
        int bytesWritten=uart_write_bytes(UART_NUM_2, (const char*)midiMess, sizeof(midiMess));
        if(bytesWritten != sizeof(midiMess)){
            printf("Error writing to UART\n");
        }else{
            printf("Data sent to UART \n");
        }
    }

    free(data);
    return 0;
}



static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    
    case BLE_GAP_EVENT_CONNECT:
        
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }else{
            //blinkaj LED prilikom uspostave konekcije
            gpio_set_level(LED_GPIO_PIN,1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(LED_GPIO_PIN,0);
        }
        break;
    
    case BLE_GAP_EVENT_DISCONNECT:
        ble_app_advertise();
        
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// BLE konekcija
void ble_app_advertise(void)
{
    
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; 
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; 
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}


void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); 
    ble_app_advertise();                     
}


void host_task(void *param)
{
    nimble_port_run(); 
}

void app_main()
{
        printf("Hello!");
        gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

       

        // UART config
        uart_config_t uart_config = {
        .baud_rate = 31250,  // MIDI baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    // UART driver
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    
    gpio_config(&io_conf);
    
    nvs_flash_init();                          
    
    nimble_port_init();                        
    ble_svc_gap_device_name_set("MIDI_Controller"); 
    ble_svc_gap_init();                        
    ble_svc_gatt_init();                       
    ble_gatts_count_cfg(gatt_svcs);            
    ble_gatts_add_svcs(gatt_svcs);             
    ble_hs_cfg.sync_cb = ble_app_on_sync;      
    nimble_port_freertos_init(host_task);      
}
