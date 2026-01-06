
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tusb.h>
#include <pico/stdlib.h>
#include "usbSerialDebug/helper.h"
#include <FreeRTOS.h>
#include <task.h>
#include "tkjhat/sdk.h"
#include <math.h>
#include <queue.h>

#define BUFFER_SIZE         64
#define DEFAULT_STACK_SIZE  2048
#define CDC_ITF_TX          1

// Globaalit muuttujat
TaskHandle_t hUSB = NULL, hRotationTask = NULL, hTransmitter = NULL, hBuzzer = NULL, hMusic = NULL;
QueueHandle_t buzzerQueue = NULL;
int consecutiveSpaces = 0;
int rotation_first = 0;
int acceleration_first = 0;

//Viestin kirjoittamista varten
char message[BUFFER_SIZE] = "";

//Tilamuuttujat kirjoittamista, lähettämistä ja vastaanottoa varten
enum state {WRITING = 1, TRANSMITTING, RECEIVING};
enum state programState = WRITING;

// Button 
void buttonTask(uint gpio, uint32_t eventMask) {

    // estetään kirjoitus vastaanoton aikana
    if (programState == RECEIVING){
        return; 
    } 

    if (gpio == BUTTON1 && programState == WRITING) {
        // Lisätään '-' message arrayihin
        size_t len = strlen(message);
        if (len < BUFFER_SIZE - 1) {
            message[len] = '-';
            message[len + 1] = '\0';
            consecutiveSpaces = 0; // reset
        }
        usb_serial_print("Button pressed! \n");

    }

    if (gpio == BUTTON2 && programState == WRITING) {
        // Lisätään ' ' message arrayihin
        size_t len = strlen(message);
        if (len < BUFFER_SIZE - 1) {
            message[len] = ' ';
            message[len + 1] = '\0';
        }
        usb_serial_print("Button pressed! \n");

        // Välien laskeminen (3 peräkkäistä lähettää viestin)
        consecutiveSpaces++;
        if (consecutiveSpaces >= 3) {
            consecutiveSpaces = 0; // reset

            // Append '\n' to the message
            size_t len = strlen(message);
            if (len < BUFFER_SIZE - 1) {
                message[len] = '\n';
                message[len + 1] = '\0';
                programState = TRANSMITTING;
            }
            // Herätetään transmitter
            if (hTransmitter != NULL) {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                vTaskNotifyGiveFromISR(hTransmitter, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

// IMU 

static void rotation_task(void *arg) {
    (void)arg;

    float ax, ay, az, gx, gy, gz, t;
    // Alustus 
    if (init_ICM42670() == 0) {
        usb_serial_print("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            usb_serial_print("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        usb_serial_print("Failed to initialize ICM-42670P.\n");
    }

    uint8_t buf[BUFFER_SIZE];
    
    while (1) {
        // estetään kirjoitus vastaanoton aikana
        if (programState == RECEIVING){
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
            }

        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0 && programState == WRITING) {

            // Kiihtymisen havainnointi
            if (fabs(ax) > 0.8 ) { // Arvo kiihtymisen tunnistamiseen
                usb_serial_print("Acceleration detected!\n");
                if (acceleration_first == 0) {
                    acceleration_first = 1;
                } else {
                    // Lisätään '-' message arrayihin
                    size_t len = strlen(message);
                    if (len < BUFFER_SIZE - 1) {
                        message[len] = '-';
                        message[len + 1] = '\0';
                        consecutiveSpaces = 0; // reset
                    }
                }
            }

            // Rotaation havainnointi
            if (fabs(gx) > 110) { // Arvo rotaation tunnistamiseen
                usb_serial_print("Rotation detected!\n");
                if (rotation_first == 0) {
                    rotation_first = 1;
                } else {
                    // Lisätään '.' message arrayihin
                    size_t len = strlen(message);
                    if (len < BUFFER_SIZE - 1) {
                        message[len] = '.';
                        message[len + 1] = '\0';
                        consecutiveSpaces = 0; // reset
                    }
                }
            }
            sprintf(buf, "Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f \n", ax, ay, az, gx, gy, gz);
            usb_serial_print(buf);
        } else {
            usb_serial_print("Failed to read imu data\n");

        }
        // Näytä merkit näytöllä
        clear_display();
        write_text(message);

        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

// Morse Transmitter 

void transmitter(void *arg) {
    (void)arg;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (tud_cdc_n_connected(CDC_ITF_TX) && programState == TRANSMITTING) {
            tud_cdc_n_write(CDC_ITF_TX, message, strlen(message));
            tud_cdc_n_write_flush(CDC_ITF_TX);
            // Palataan kirjoitus-tilaan
            programState = WRITING;
        }
        // Äänimerkki
        xQueueSend(buzzerQueue, &(int){1}, 0);

        // Message-bufferin resetointi
        memset(message, 0, sizeof(message));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Morse Receiver 
// Funktion nimeä ei ilmeisesti saanut muuttaa
void tud_cdc_rx_cb(uint8_t itf){
    // Estetään muut taskit vastaanoton aikana
    programState = RECEIVING; 

    // varaa puskuri datalle pinosta
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE + 1];

    // lue saatavilla oleva data
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

    // tarkista, tuliko data toisesta CDC-liittymästä
    if (itf == 1) {
         // Voit myös lähettää sen takaisin CDC:hen:
        tud_cdc_n_write(itf, buf, count);
        tud_cdc_n_write_flush(itf);
    }

    // RGB LED alustus ja päälle
    init_rgb_led();
    set_led_status(true);
    // Annetaan bufferin merkeille eri värit
    for (uint32_t i = 0; i < count; i++) {
        char c = buf[i];

        if (c == '.') {
            rgb_led_write(255, 0, 0);  
        } 
        else if (c == '-') {
            rgb_led_write(0, 255, 0);  
        } 
        else if (c == ' ') {
            rgb_led_write(0, 0, 255);  
        } 
        else if (c == '\n') {
            xQueueSend(buzzerQueue, &(int){2}, 0);;
            
            continue;
        }

        // Näytä merkit näytöllä
        clear_display();
        write_text((char*)buf);
        vTaskDelay(pdMS_TO_TICKS(500));

        // LED pois päältä
        stop_rgb_led(); 
    }
    // Palataan kirjoitus-tilaan
    programState = WRITING;
}


// Buzzer

static void buzzer_task(void *arg) {
    (void)arg;
    // Alustetaan buzzer
    init_buzzer();
    int toneCode;

    //Annetaan eri lähetykselle ja vastaanotolle eri äänet
    while (1) {
        if (xQueueReceive(buzzerQueue, &toneCode, portMAX_DELAY)) {
            switch (toneCode) {
                case 1: // transmission
                    buzzer_play_tone(440, 500);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    buzzer_play_tone(494, 500);
                    break;

                case 2: // receiver
                    buzzer_play_tone(523, 250);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    buzzer_play_tone(500, 250);
                    break;

                case 3: // Musiikki
                    {
                        // "Super Mario Bros. theme" -melodia
                        // Melodian nuotit tämän sivuston avulla: 
                        // https://theorycircuit.com/esp32-projects/super-mario-melody-tone-using-esp32-and-buzzer/

                        int notes[] = {440, 440, 0, 440, 0, 349, 440, 0, 523, 0};

                        int durations[] = {80, 80, 80, 80, 80, 80, 80, 80, 40, 40};

                        int len = sizeof(notes) / sizeof(notes[0]);

                        for (int i = 0; i < len; i++) {
                            buzzer_play_tone(notes[i], durations[i]);
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }
                    }
                    break;
            }
        }
    }
}

static void music_task(void *arg) {
    (void)arg;

    
    while (1) {
        // Estetään musiikkia soimasta vastaanoton aikana
        if (programState == RECEIVING){
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
            }
        

        if (programState == WRITING){
        // Odota 30 sekuntia
        vTaskDelay(pdMS_TO_TICKS(30000));

        // Soita musiikkia
        xQueueSend(buzzerQueue, &(int){3}, 0);

        usb_serial_print("Played music!\n");
        }
    }
}

static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}

int main(){
    
    //Alustuksia
    init_hat_sdk();
    init_display();
    sleep_ms(1000); 
    buzzerQueue = xQueueCreate(5, sizeof(int));

    // Nappien alustukset
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);

    // Keskeytykset napeille
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, buttonTask);
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_FALL, true, buttonTask);


    // Luodaan taskit
    xTaskCreate(transmitter, "transmitter", DEFAULT_STACK_SIZE, NULL, 2, &hTransmitter);
    xTaskCreate(usbTask, "usb", DEFAULT_STACK_SIZE, NULL, 3, &hUSB);
    xTaskCreate(rotation_task, "rotation", DEFAULT_STACK_SIZE, NULL, 2, &hRotationTask);
    xTaskCreate(buzzer_task, "buzzer", DEFAULT_STACK_SIZE, NULL, 2, &hBuzzer);
    xTaskCreate(music_task, "music", DEFAULT_STACK_SIZE, NULL, 2, &hMusic);

    //Debugaamista varten
    tusb_init();

    // Aloita FreeRTOS scheduler
    usb_serial_init();
    vTaskStartScheduler();

    return 0;
}