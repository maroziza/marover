#include <stdio.h>
#include <string.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <duktape.h>
#include "esp_event.h"
#include "esp_system.h"
#include "esp32-js-log.h"

#include "esp_log.h"
#include "esp_newlib.h"
#include "freertos/timers.h"
//#include "duk_module_node.h"
#include "esp32-javascript.h"
static const char *tag = "esp32-javascript";


// global task handle
TaskHandle_t task;
xQueueHandle el_event_queue;

bool flag = false;
duk_context *esp32JS = NULL;

// only for debug purposes
bool DISABLE_EVENTS = false;

void fileLog(duk_context *esp32JS, log_level_t level, char *message)
{
    if (level >= INFO)
    {
        // append to logfile
        duk_eval_string(esp32JS, "typeof el_appendLogBuffer");
        if (strcmp(duk_get_string(esp32JS, -1), "function") == 0)
        {
            duk_push_string(esp32JS, "el_appendLogBuffer");
            duk_eval(esp32JS);
            duk_push_string(esp32JS, message);
            duk_push_uint(esp32JS, level);
            duk_call(esp32JS, 2);
        }
    }
}

void jslog(log_level_t level, const char *msg, ...)
{
    if (level < (5 - LOG_LOCAL_LEVEL))
    {
        return;
    }

    char *my_string;
    va_list argp;

    va_start(argp, msg);
    vasprintf(&my_string, msg, argp);
    va_end(argp);

    TaskHandle_t current = xTaskGetCurrentTaskHandle();

    if (esp32JS && current == task) // prevent race conditions with esp32JS from different tasks
    {
        if (level == DEBUG)
        {
            duk_push_string(esp32JS, "console.debug");
        }
        else if (level == INFO)
        {
            duk_push_string(esp32JS, "console.info");
        }
        else if (level == WARN)
        {
            duk_push_string(esp32JS, "console.warn");
        }
        else
        {
            duk_push_string(esp32JS, "console.error");
        }
        duk_eval(esp32JS); /* -> [ ... func ] */
        duk_push_string(esp32JS, my_string);
        duk_call(esp32JS, 1);
        //clear stack
        duk_pop(esp32JS);
    }
    else
    {
        char *message = spiram_malloc(strlen(my_string) + 1); // gets freed in el_readAndFreeString
        if (message == NULL)
        {
            ESP_LOGE(tag, "No memory to log message. Aborting...");
            abort();
        }

        strcpy(message, my_string);

        js_event_t event;
        js_eventlist_t events;
        events.events_len = 0;

        el_create_event(&event, EL_LOG_EVENT_TYPE, level, message);
        el_add_event(&events, &event);
        el_fire_events(&events);
    }
    free(my_string);
}

void IRAM_ATTR el_add_event(js_eventlist_t *events, js_event_t *event)
{
    if (events->events_len >= MAX_EVENTS)
    {
        ESP_LOGE(tag, "Event list is full. Max event list size: %d => aborting.", MAX_EVENTS);
        abort();
    }
    events->events[events->events_len] = *event;
    events->events_len = events->events_len + 1;
}

void IRAM_ATTR el_fire_events(js_eventlist_t *events)
{
    if (DISABLE_EVENTS)
    {
        ESP_LOGW(tag, "Events are disabled. They will never be fired.");
    }
    else
    {
        if (events->events_len > 0)
        {
            // el_fire_events is called from ISR and logging leads to overflowing stack (sometimes)
            // e.g. in timer routines, because timer task has limited stack size. Discovered in
            // backtrace of ELF 00d57bf2ccbe7755
            // ESP_LOGD(tag, "Send %d events to queue...", events->events_len);
            int ret = xQueueSendFromISR(el_event_queue, events, NULL);
            if (ret != pdTRUE)
            {
                ESP_LOGE(tag, "Event queue is full... is something blocking the event loop?...aborting.");
                js_eventlist_t devents;
                int num = 0;
                while (xQueueReceive(el_event_queue, &devents, 0))
                {
                    for (int i = 0; i < devents.events_len; i++)
                    {
                        ESP_LOGE(tag, "Events num %i, event idx %i, type %i, status %i", num, i, devents.events[i].type, devents.events[i].status);
                    }
                    num++;
                }

                abort();
            }
        }
    }
}

void IRAM_ATTR el_create_event(js_event_t *event, int type, int status, void *fd)
{
    event->type = type;
    event->status = status;
    event->fd = fd;
}

void IRAM_ATTR vTimerCallback(TimerHandle_t xTimer)
{
    js_event_t event;
    js_eventlist_t events;

    el_create_event(&event, EL_TIMER_EVENT_TYPE, (int)xTimer, 0);
    events.events_len = 0;
    el_add_event(&events, &event);
    el_fire_events(&events);
}

int createTimer(int timer_period_us)
{
    int interval = timer_period_us / portTICK_PERIOD_MS;

    TimerHandle_t tmr = xTimerCreate("", interval <= 0 ? 1 : interval, pdFALSE, NULL, vTimerCallback);

    if (tmr == NULL)
    {
        jslog(ERROR, "Could not create timer.");
        abort();
    }

    if (interval <= 0)
    {
        // fire event immediatley without starting the timer
        vTimerCallback(tmr);
    }
    else
    {
        if (xTimerStart(tmr, portMAX_DELAY) != pdPASS)
        {
            jslog(ERROR, "Timer start error");
        }
    }
    return (int)tmr;
}

static duk_ret_t native_delay(duk_context *esp32JS)
{
    int delay = duk_to_int32(esp32JS, 0);
    jslog(DEBUG, "Waiting %dms...", delay);
    if (delay < 0)
    {
        delay = 0;
    }
    vTaskDelay(delay / portTICK_PERIOD_MS);
    return 0;
}

static duk_ret_t el_createTimer(duk_context *esp32JS)
{
    int delay = duk_to_int32(esp32JS, 0);
    if (delay < 0)
    {
        delay = 0;
    }
    jslog(DEBUG, "Install timer to notify in  %dms.", delay);
    int handle = createTimer(delay);
    duk_push_int(esp32JS, handle);
    return 1;
}

static duk_ret_t el_removeTimer(duk_context *esp32JS)
{
    int handle = duk_to_int32(esp32JS, 0);
    xTimerDelete((TimerHandle_t)handle, portMAX_DELAY);
    return 0;
}


static duk_ret_t el_suspend(duk_context *esp32JS)
{
    // feed watchdog
    vTaskDelay(1);

    // jslog(INFO, "Free memory: %d bytes", esp_get_free_heap_size());
    js_eventlist_t events;

    jslog(DEBUG, "Waiting for events...");
    int arr_idx = duk_push_array(esp32JS);
    int arrsize = 0;
    TickType_t timeout = portMAX_DELAY;

    // force garbage collection 2 times see duktape doc
    // greatly increases perfomance with external memory
    duk_gc(esp32JS, 0);
    duk_gc(esp32JS, 0);

    while (xQueueReceive(el_event_queue, &events, timeout) == pdTRUE)
    {
        timeout = 0; // set timeout to 0 to not wait in while loop if there are no more events available
        for (int i = 0; i < events.events_len; i++)
        {
            duk_idx_t obj_idx = duk_push_object(esp32JS);

            duk_push_int(esp32JS, events.events[i].type);
            duk_put_prop_string(esp32JS, obj_idx, "type");
            duk_push_int(esp32JS, events.events[i].status);
            duk_put_prop_string(esp32JS, obj_idx, "status");
            duk_push_int(esp32JS, (int)events.events[i].fd);
            duk_put_prop_string(esp32JS, obj_idx, "fd");

            duk_put_prop_index(esp32JS, arr_idx, arrsize);
            arrsize++;
        }
    }

    jslog(DEBUG, "Received %d events.", arrsize);

    return 1;
}

static duk_ret_t info(duk_context *esp32JS)
{
    size_t internal = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    size_t external = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);

    jslog(INFO, "INTERNAL MEMORY HEAP INFO FREE: %d", internal);
    jslog(INFO, "EXTERNAL MEMORY HEAP INFO FREE: %d", external);

    return 0;
}

static duk_ret_t el_restart(duk_context *esp32JS)
{
    esp_restart();
    return 0;
}


static duk_ret_t setDateTimeInMillis(duk_context *esp32JS)
{
    double timeInMillis = duk_to_number(esp32JS, 0);
    struct timeval tv;
    tv.tv_sec = (time_t)(timeInMillis / (double)1000.0);
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    return 0;
}

static duk_ret_t setDateTimeZoneOffsetInHours(duk_context *esp32JS)
{
    uint16_t offset = duk_to_uint16(esp32JS, 0);
    duk_dateTimeZoneOffsetInHours = offset;
    return 0;
}

void loadJS(duk_context *esp32JS, const char *name, char *start, char *end)
{
    const unsigned int length = end - start - 1;
    jslog(INFO, "Loading %s ...", name);
    duk_eval_lstring_noresult(esp32JS, start, length);
}


duk_ret_t el_readAndFreeString(duk_context *esp32JS)
{
    char *str = duk_to_int(esp32JS, 0);
    duk_push_string(esp32JS, str);
    spiram_free(str);
    return 1;
}

void duktape_task(void *ignore)
{
	esp32JS = jsCreateContext();
    createConsole(esp32JS);


    jslog(INFO, "Free memory: %d bytes", esp_get_free_heap_size());

   
#ifdef KEY_BUILTIN
    duk_push_int(esp32JS, KEY_BUILTIN);
#else
    duk_push_undefined(esp32JS);
#endif
    duk_put_global_string(esp32JS, "KEY_BUILTIN");

#ifdef LED_BUILTIN
    duk_push_int(esp32JS, LED_BUILTIN);
#else
    duk_push_undefined(esp32JS);
#endif
    duk_put_global_string(esp32JS, "LED_BUILTIN");


    duk_push_c_function(esp32JS, info, 0 /*nargs*/);
    duk_put_global_string(esp32JS, "info");

    duk_push_c_function(esp32JS, el_suspend, 0 /*nargs*/);
    duk_put_global_string(esp32JS, "el_suspend");

    duk_push_c_function(esp32JS, el_createTimer, 1 /*nargs*/);
    duk_put_global_string(esp32JS, "el_createTimer");

    duk_push_c_function(esp32JS, el_removeTimer, 1 /*nargs*/);
    duk_put_global_string(esp32JS, "el_removeTimer");

   
    duk_push_c_function(esp32JS, el_restart, 0 /*nargs*/);
    duk_put_global_string(esp32JS, "restart");


    duk_push_c_function(esp32JS, setDateTimeInMillis, 1 /*nargs*/);
    duk_put_global_string(esp32JS, "setDateTimeInMillis");

    duk_push_c_function(esp32JS, setDateTimeZoneOffsetInHours, 1 /*nargs*/);
    duk_put_global_string(esp32JS, "setDateTimeZoneOffsetInHours");


    duk_push_int(esp32JS, EL_TIMER_EVENT_TYPE);
    duk_put_global_string(esp32JS, "EL_TIMER_EVENT_TYPE");

    duk_push_int(esp32JS, EL_LOG_EVENT_TYPE);
    duk_put_global_string(esp32JS, "EL_LOG_EVENT_TYPE");

    duk_push_c_function(esp32JS, el_readAndFreeString, 1 /*nargs*/);
    duk_put_global_string(esp32JS, "el_readAndFreeString");

//    loadUrlPolyfill(esp32JS);

#define ESP32_JAVASCRIPT_EXTERN ESP32_JAVASCRIPT_EXTERN_REGISTER
//#include "esp32-javascript-config.h"
#undef ESP32_JAVASCRIPT_EXTERN

    duk_eval_string_noresult(esp32JS, "require('esp32-javascript')");

#define ESP32_JAVASCRIPT_EXTERN ESP32_JAVASCRIPT_EXTERN_LOAD
//#include "esp32-javascript-config.h"
#undef ESP32_JAVASCRIPT_EXTERN

    jslog(INFO, "Reaching end of event loop.");

    //Return from task is not allowed
    vTaskDelete(NULL);
}

void esp32_javascript_init() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("dhcpc", ESP_LOG_WARN);
    esp_log_level_set(tag, LOG_LOCAL_LEVEL);

    tcpip_adapter_init();

    el_event_queue = xQueueCreate(256, sizeof(js_eventlist_t));
    jslog(INFO, "Free memory: %d bytes", esp_get_free_heap_size());

    xTaskCreatePinnedToCore(&duktape_task, "duktape_task", 24 * 1024, NULL, 5, &task, 0);
}
