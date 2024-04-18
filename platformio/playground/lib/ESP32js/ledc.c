#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "esp32-js-log.h"
#include "pins_arduino.h"
#include "duk_config.h"
#include "duktape.h"



static duk_ret_t el_pinMode(duk_context *ctx)
{
    int pin = duk_to_int(ctx, 0);
    int dir = duk_to_int(ctx, 1);

    jslog(DEBUG, "el_pinMode pin=%d dir=%d", pin, dir);

    pinMode(pin, dir);
    return 0;
}

static duk_ret_t el_digitalWrite(duk_context *ctx)
{
    int pin = duk_to_int(ctx, 0);
    int level = duk_to_int(ctx, 1);

    jslog(DEBUG, "el_digitalWrite pin=%d level=%d", pin, level);

    digitalWrite(pin, level);
    return 0;
}

static duk_ret_t el_digitalRead(duk_context *ctx)
{
    int pin = duk_to_int(ctx, 0);

    jslog(DEBUG, "el_digitalRead pin=%d", pin);

    int val = digitalRead(pin);
    duk_push_int(ctx, val);
    return 1;
}

static duk_ret_t el_ledcSetup(duk_context *ctx)
{
    int channel = duk_to_int(ctx, 0);
    int freq = duk_to_int(ctx, 1);
    int resolution = duk_to_int(ctx, 2);

    jslog(DEBUG, "el_ledcSetup channel=%d freq=%d resolution=%d ", channel, freq, resolution);

    ledcSetup(channel, freq, resolution);
    return 0;
}

static duk_ret_t el_ledcAttachPin(duk_context *ctx)
{
    int pin = duk_to_int(ctx, 0);
    int channel = duk_to_int(ctx, 1);

    jslog(DEBUG, "el_ledcAttachPin pin=%d channel=%d", pin, channel);

    ledcAttachPin(pin, channel);
    return 0;
}

static duk_ret_t el_ledcWrite(duk_context *ctx)
{
    int channel = duk_to_int(ctx, 0);
    int dutyCycle = duk_to_int(ctx, 1);

    jslog(DEBUG, "el_ledcWrite channel=%d dutyCycle=%d ", channel, dutyCycle);

    ledcWrite(channel, dutyCycle);
    return 0;
}



void ledc_module_init(duk_context *ctx) {
	#ifdef KEY_BUILTIN
    duk_push_int(ctx, KEY_BUILTIN);
#else
    duk_push_undefined(ctx);
#endif
    duk_put_global_string(ctx, "KEY_BUILTIN");

#ifdef LED_BUILTIN
    duk_push_int(ctx, LED_BUILTIN);
#else
    duk_push_undefined(ctx);
#endif
    duk_put_global_string(ctx, "LED_BUILTIN");


	
	
	duk_push_c_function(ctx, el_pinMode, 2 /*nargs*/);
    duk_put_global_string(ctx, "pinMode");

    duk_push_c_function(ctx, el_digitalRead, 1 /*nargs*/);
    duk_put_global_string(ctx, "digitalRead");

    duk_push_c_function(ctx, el_digitalWrite, 2 /*nargs*/);
    duk_put_global_string(ctx, "digitalWrite");

    duk_push_int(ctx, 1);
    duk_put_global_string(ctx, "HIGH");

    duk_push_int(ctx, 0);
    duk_put_global_string(ctx, "LOW");

    duk_push_int(ctx, INPUT);
    duk_put_global_string(ctx, "INPUT");

    duk_push_int(ctx, OUTPUT);
    duk_put_global_string(ctx, "OUTPUT");

	
	duk_push_c_function(ctx, el_ledcSetup, 3 /*nargs*/);
    duk_put_global_string(ctx, "ledcSetup");

    duk_push_c_function(ctx, el_ledcAttachPin, 2 /*nargs*/);
    duk_put_global_string(ctx, "ledcAttachPin");

    duk_push_c_function(ctx, el_ledcWrite, 2 /*nargs*/);
    duk_put_global_string(ctx, "ledcWrite");

	
}
