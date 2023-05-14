#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <STM32duinoBLE.h>

#define SERVICE_UUID                        "19B10010-E8F2-537E-4F6C-D104768A1214"
#define TEMPERATURE_CHARACTERISTIC_UUID     "19B10011-E8F2-537E-4F6C-D104768A1214"
#define HUMIDITY_CHARACTERISTIC_UUID        "19B10012-E8F2-537E-4F6C-D104768A1214"
#define EVENT_FLAG_MEASSURE                 0
#define DHT11_TEMPERATURE_IS_ENABLED        0
#define DHT11_HUMIDITY_IS_ENABLED           1

typedef enum {START, ZERO, ONE} bit_t;
typedef enum {DELAY, READ} timer_mode_t;
typedef struct {
  int16_t temperature;
  int16_t humidity;
} sensor_data_t;

const PinName dht11_pin_name = PA_7_ALT1;
const int dht11_pin = PA7;

HardwareTimer *timer = nullptr;
QueueHandle_t queue_ble, queue_dht11_timestamp;
volatile timer_mode_t mode;

HCISharedMemTransportClass hci_shared_mem_transport;
#if !defined(FAKE_BLELOCALDEVICE)
    BLELocalDevice ble_obj(&hci_shared_mem_transport);
    BLELocalDevice& BLE = ble_obj;
#endif

BLEService dht11_service(SERVICE_UUID);
BLEShortCharacteristic temperature_characteristic(TEMPERATURE_CHARACTERISTIC_UUID, BLERead | BLENotify);
BLEShortCharacteristic humidity_characteristic(HUMIDITY_CHARACTERISTIC_UUID, BLERead | BLENotify);
TimerHandle_t timer_dht11;
EventGroupHandle_t event_group;
uint8_t measurement_flags = 0;

void handle_error();
void timer_ic();
void timer_ov();
void configure_delay();
void configure_read();
void setup_timer();
void dht11_measurement(uint16_t *temperature, uint16_t *humidity);

void timer_ic() {  
  if (mode == READ) {
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(dht11_pin_name, PinMap_TIM));
    uint16_t value = timer->getCaptureCompare(channel);
	  BaseType_t higher_priority_task_woken = pdFALSE;
    if (xQueueSendFromISR(queue_dht11_timestamp, &value, &higher_priority_task_woken) != pdPASS)
        handle_error();
  }
}

void timer_ov() {
  if (mode == DELAY) {
    configure_read();
  }
}

void configure_delay() {
  mode = DELAY;
  timer->pause();
  timer->refresh();
  pinMode(dht11_pin, OUTPUT);
  digitalWrite(dht11_pin, LOW);
  timer->resume();
}

void configure_read() {
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(dht11_pin_name, PinMap_TIM));
  pinMode(dht11_pin, INPUT_PULLUP);
  pin_function(dht11_pin_name, STM_PIN_DATA_EXT(STM_MODE_AF_OD, GPIO_PULLUP, GPIO_AF14_TIM17, channel, 0));
  mode = READ;
}

void setup_timer() {
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(dht11_pin_name, PinMap_TIM);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(dht11_pin_name, PinMap_TIM));
  timer = new HardwareTimer(Instance);
  timer->pause();
  timer->setPrescaleFactor(32);
  timer->setOverflow(18000);
  timer->attachInterrupt(channel, timer_ic);
  timer->attachInterrupt(timer_ov);
  timer->setMode(channel, TIMER_INPUT_CAPTURE_BOTHEDGE, NC);
  timer->refresh();
}

void dht11_measurement(int16_t *temperature, int16_t *humidity) {
	uint16_t value;
	bool first = true, started = false;
	uint32_t last = 0;
	uint32_t low_portion = 0;
	uint8_t val = 0, humidity1, humidity2, temperature1, temperature2;
	uint32_t count = 0;

	configure_delay();

	while(count < 40) {
		if (xQueueReceive(queue_dht11_timestamp, &value, portMAX_DELAY) != pdPASS)
			handle_error();

		if (first) {
			first = false;
			last = value;
		} else {
			uint32_t duration;
			if (value < last)
				duration = value - (last - 18000);
			else
				duration = value - last;
			if (duration == 0)
				handle_error();

			last = value;
			if (low_portion == 0)
				low_portion = duration;
			else {
				bit_t bit;

				if (low_portion > 70 && low_portion < 90 && duration > 70 && duration < 90)
					bit = START;
				else if (low_portion > 40 && low_portion < 60 && duration > 20 && duration < 35)
					bit = ZERO;
				else if (low_portion > 40 && low_portion < 60 && duration > 60 && duration < 80)
					bit = ONE;
				else {
					handle_error();
				}
				low_portion = 0;

				switch(bit) {
					case START:
						started = true;
						break;
					case ZERO:
						val <<= 1;
						count ++;
						break;
					case ONE:
						val <<= 1;
						val |= 1;
						count ++;
						break;
				}

				if (started) {
					if (count == 8) {
						humidity1 = val;
						val = 0;
					} else if (count == 16) {
						humidity2 = val;
						val = 0;
					} else if (count == 24) {
						temperature1 = val;
						val = 0;
					} else if (count == 32) {
						temperature2 = val;
						val = 0;
					} else if (count == 40) {
						if (humidity1 + humidity2 + temperature1 + temperature2 != val)
							handle_error();
						else {
							*humidity = (humidity1 << 8) | humidity2;
							*temperature = (temperature1 << 8) | temperature2;
						}
					}
				}
			}
		}
	}
	if (!started)
		handle_error();
	timer->pause();
	while (xQueueReceive(queue_dht11_timestamp, &value, 0) == pdPASS); //Drain timestamp queue
}

void timer_counter_callback(TimerHandle_t timer) {
  xEventGroupSetBits(event_group, (1 << EVENT_FLAG_MEASSURE));
}

void ble_temperature_characteristic_connect_handler(BLEDevice central, BLECharacteristic characteristic) {  
  if(xTimerStart(timer_dht11, 0) != pdPASS)
    handle_error();
  measurement_flags |= (1 << DHT11_TEMPERATURE_IS_ENABLED);
}

void ble_temperature_characteristic_disconnect_handler(BLEDevice central, BLECharacteristic characteristic) {
  measurement_flags &= ~(1 << DHT11_TEMPERATURE_IS_ENABLED);
  if ((measurement_flags & ((1 << DHT11_TEMPERATURE_IS_ENABLED) | (1 << DHT11_HUMIDITY_IS_ENABLED))) == 0)
    if(xTimerStop(timer_dht11, 0) != pdPASS)
      handle_error();
}

void ble_humidity_characteristic_connect_handler(BLEDevice central, BLECharacteristic characteristic) {
  if(xTimerStart(timer_dht11, 0) != pdPASS)
    handle_error();
  measurement_flags |= (1 << DHT11_HUMIDITY_IS_ENABLED);
}

void ble_humidity_characteristic_disconnect_handler(BLEDevice central, BLECharacteristic characteristic) {
  measurement_flags &= ~(1 << DHT11_HUMIDITY_IS_ENABLED);
  if ((measurement_flags & ((1 << DHT11_TEMPERATURE_IS_ENABLED) | (1 << DHT11_HUMIDITY_IS_ENABLED))) == 0)
    if(xTimerStop(timer_dht11, 0) != pdPASS)
      handle_error();
}

void task_ble(void *args) {
  sensor_data_t data;

  if (!BLE.begin()) {
    handle_error();
  }
  BLE.setLocalName("tpolzer_pio");
  BLE.setAdvertisedService(dht11_service);
  dht11_service.addCharacteristic(temperature_characteristic);
  dht11_service.addCharacteristic(humidity_characteristic);
  BLE.addService(dht11_service);
  temperature_characteristic.setEventHandler(BLEConnected, ble_temperature_characteristic_connect_handler);
  temperature_characteristic.setEventHandler(BLEDisconnected, ble_temperature_characteristic_disconnect_handler);
  humidity_characteristic.setEventHandler(BLEConnected, ble_humidity_characteristic_connect_handler);
  humidity_characteristic.setEventHandler(BLEDisconnected, ble_humidity_characteristic_disconnect_handler);
  BLE.advertise();

  for (;;) {
    BLE.poll();
    if (xQueueReceive(queue_ble, &data, 0) == pdPASS) {
      if ((measurement_flags & (1 << DHT11_TEMPERATURE_IS_ENABLED)) != 0)
        temperature_characteristic.writeValue(data.temperature);
      if ((measurement_flags & (1 << DHT11_HUMIDITY_IS_ENABLED)) != 0)
        humidity_characteristic.writeValue(data.humidity);
    }
    vTaskDelay(100L / portTICK_PERIOD_MS);
  }
}

void task_dht11(void *args) {
  setup_timer();

  while (true) {
    EventBits_t bits = xEventGroupWaitBits(event_group, 1 << EVENT_FLAG_MEASSURE, pdTRUE, pdFALSE, portMAX_DELAY);
    if ((bits & (1 << EVENT_FLAG_MEASSURE)) != 0) {
      int16_t temperature = INT16_MIN, humidity = INT16_MIN;
      dht11_measurement(&temperature, &humidity);
      if (temperature > INT16_MIN && humidity  > INT16_MIN) {
        sensor_data_t data;
        data.temperature = temperature;
        data.humidity = humidity;
        if (xQueueSend(queue_ble, &data, portMAX_DELAY) != pdPASS)
          handle_error();
      }
    }
  }
}

void setup() {
  pinMode(LED_RED, OUTPUT);
  if ((event_group = xEventGroupCreate()) == NULL)
    handle_error();
  if (xTaskCreate(task_ble, "Task BLE", configMINIMAL_STACK_SIZE + 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    handle_error();
  if(xTaskCreate(task_dht11, "Task Measure DHT11", configMINIMAL_STACK_SIZE + 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
  	handle_error();
  if ((timer_dht11 = xTimerCreate("Timer Counter", 5000 / portTICK_PERIOD_MS, pdTRUE, (void *) 0, timer_counter_callback)) == NULL)
    handle_error();
  if ((queue_ble = xQueueCreate(8, sizeof(sensor_data_t))) == NULL)
	  handle_error();
  if ((queue_dht11_timestamp = xQueueCreate(256, sizeof(uint16_t))) == NULL)
  	handle_error();
  vTaskStartScheduler();
  handle_error();
}

void loop() {
  // FreeRTOS idle loop
  // No blocking operations (including vTaskDelay) allowed
  //vTaskDelay(1000L / portTICK_PERIOD_MS);
}

void handle_error() {
  noInterrupts();
  digitalWrite(LED_RED, HIGH);
  for(;;);
}
