#include "duk_config.h"
#include "duktape.h"
#include "esp32-js-log.h"
#include "esp_log.h"
#include "esp32-javascript.h"


static const char *tag = "esp32-javascript";


static duk_ret_t console_debug_binding(duk_context *ctx)
{
    char *message = duk_to_string(ctx, 0);
    ESP_LOGD(tag, "%s", message);
    fileLog(ctx, DEBUG, message);
    return 0;
}
static duk_ret_t console_info_binding(duk_context *ctx)
{
    char *message = duk_to_string(ctx, 0);
    ESP_LOGI(tag, "%s", message);
    fileLog(ctx, INFO, message);
    return 0;
}
static duk_ret_t console_warn_binding(duk_context *ctx)
{
    char *message = duk_to_string(ctx, 0);
    ESP_LOGW(tag, "%s", message);
    fileLog(ctx, WARN, message);
    return 0;
}
static duk_ret_t console_error_binding(duk_context *ctx)
{
    char *message = duk_to_string(ctx, 0);
    ESP_LOGE(tag, "%s", message);
    fileLog(ctx, ERROR, message);
    return 0;
}


void createConsole(duk_context *ctx)
{
    duk_idx_t obj_idx = duk_push_object(ctx);

    duk_push_c_function(ctx, console_info_binding, 1);
    duk_put_prop_string(ctx, obj_idx, "log");

    duk_push_c_function(ctx, console_debug_binding, 1);
    duk_put_prop_string(ctx, obj_idx, "debug");
    
    duk_push_c_function(ctx, console_info_binding, 1);
    duk_put_prop_string(ctx, obj_idx, "info");
  
    duk_push_c_function(ctx, console_warn_binding, 1);
    duk_put_prop_string(ctx, obj_idx, "warn"); 
  
    duk_push_c_function(ctx, console_error_binding, 1);
    duk_put_prop_string(ctx, obj_idx, "error");


    duk_push_c_function(ctx, console_info_binding, 1 /*nargs*/);
    duk_put_global_string(ctx, "print");



    duk_push_boolean(ctx, DEBUG >= (5 - LOG_LOCAL_LEVEL));
    duk_put_prop_string(ctx, obj_idx, "isDebug");
    duk_push_boolean(ctx, INFO >= (5 - LOG_LOCAL_LEVEL));
    duk_put_prop_string(ctx, obj_idx, "isInfo");
    duk_push_boolean(ctx, WARN >= (5 - LOG_LOCAL_LEVEL));
    duk_put_prop_string(ctx, obj_idx, "isWarn");
    duk_push_boolean(ctx, ERROR >= (5 - LOG_LOCAL_LEVEL));
    duk_put_prop_string(ctx, obj_idx, "isError");

   
    duk_put_global_string(ctx, "console");
}


