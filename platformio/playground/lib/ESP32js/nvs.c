#include "nvs.h"
#include "nvs_flash.h"
#include "duk_config.h"
#include <duktape.h>
#include "esp32-js-log.h"


static duk_ret_t el_load(duk_context *ctx)
{
    int ret = 0;
    esp_err_t err;

    const char *key = duk_to_string(ctx, 0);

    nvs_handle my_handle;
    err = nvs_open("esp32js2", NVS_READONLY, &my_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        duk_push_undefined(ctx);
        return 1;
    }
    else if (err != ESP_OK)
    {
        jslog(ERROR, "Error (%d) opening NVS!", err);
        return -1;
    }

    size_t string_size;
    err = nvs_get_blob(my_handle, key, NULL, &string_size);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        duk_push_undefined(ctx);
        ret = 1;
    }
    else if (err != ESP_OK)
    {
        jslog(ERROR, "Cannot get key %s from storage, err=%d", key, err);
        ret = -1;
    }
    else
    {
        char *value = (char *)malloc(string_size);
        err = nvs_get_blob(my_handle, key, value, &string_size);
        if (err != ESP_OK)
        {
            jslog(ERROR, "Cannot get key %s from storage, err=%d", key, err);
            ret = -1;
        }
        else
        {
            duk_push_string(ctx, value);
            ret = 1;
        }
        free(value);
    }
    nvs_close(my_handle);
    return ret;
}

static duk_ret_t el_store(duk_context *ctx)
{
    int ret = 0;
    esp_err_t err;

    const char *key = duk_to_string(ctx, 0);
    if (strlen(key) > 15)
    {
        jslog(ERROR, "Keys may not be longer than 15 chars. Key '%s' is longer.", key);
        return -1;
    }

    const char *value = duk_to_string(ctx, 1);
    int len = strlen(value);
    if (len > (1984 - 1))
    {
        jslog(ERROR, "Values may not be longer than 1984 chars (including zero-termination). Current string length is %d", len);
        return -1;
    }

    jslog(DEBUG, "Opening Non-Volatile Storage (NVS) ... ");
    nvs_handle my_handle;
    err = nvs_open("esp32js2", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        jslog(ERROR, "Error (%d) opening NVS!", err);
        return -1;
    }

    err = nvs_set_blob(my_handle, key, (void *)value, len + 1);
    if (err != ESP_OK)
    {
        jslog(ERROR, "Cannot set key %s and value %s from storage, err=%d", key, value, err);
        ret = -1;
    }

    err = nvs_commit(my_handle);
    if (err != ESP_OK)
    {
        jslog(ERROR, "Cannot commit changes, err=%d", err);
        ret = -1;
    }
    nvs_close(my_handle);
    return ret;
}
void nvs_module_init(duk_context *ctx) {
    nvs_flash_init();

	
    duk_push_c_function(ctx, el_load, 1 /*nargs*/);
    duk_put_global_string(ctx, "el_load");

    duk_push_c_function(ctx, el_store, 2 /*nargs*/);
    duk_put_global_string(ctx, "el_store");
}

