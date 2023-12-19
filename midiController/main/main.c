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
#define TXD_PIN (UART_PIN_NO_CHANGE)  // Use default TX pin
#define RXD_PIN (UART_PIN_NO_CHANGE)  // Use default RX pin

//char *TAG = "MIDIController";
uint8_t ble_addr_type;
void ble_app_advertise(void);

// Write data to ESP32 defined as server
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Allocate memory for null-terminated string
    char *data = (char *)malloc(ctxt->om->om_len + 1);
    if (data == NULL) {
        // Handle memory allocation failure

        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    // Copy data and null-terminate
    memcpy(data, ctxt->om->om_data, ctxt->om->om_len);
    data[ctxt->om->om_len] = '\0';
    int messType,messChnl,param1,param2;

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
            uart_write_bytes(UART_NUM_0, (const char*)midiMess, sizeof(midiMess));
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
        uart_write_bytes(UART_NUM_0, (const char*)midiMess, sizeof(midiMess));
    }


    /*if (strcmp(data, "on") == 0 && ctxt->om->om_len == 2 ) {
        uint8_t midiNoteOn[] = {0x90, 0x3C, 0x40};
        uart_write_bytes(UART_NUM_0, (const char*)midiNoteOn, sizeof(midiNoteOn));

    }else if(strcmp(data,"off")== 0 && ctxt->om->om_len == 3 ){ 
        uint8_t midiNoteOff[] = {0x80, 0x3C, 0x40};
        uart_write_bytes(UART_NUM_0, (const char*)midiNoteOff, sizeof(midiNoteOff));
    }else  {
        //printf("Data is not equal to 'eno'\n");
    }*/

    // Free allocated memory
    free(data);

        return 0;
}


// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        //ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        //ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        //ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void app_main()
{
        
        gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

       

        // Configuration for the UART communication
        uart_config_t uart_config = {
        .baud_rate = 38400,  // Standard MIDI baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    // Install the UART driver
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Configure GPIO
    gpio_config(&io_conf);
    
    nvs_flash_init();                          // 1 - Initialize NVS flash using
    // esp_nimble_hci_and_controller_init();      // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set("MIDI_Controller"); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread
}
