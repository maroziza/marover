#include "duk_config.h"
#include "duktape.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp32-js-log.h"

duk_ret_t el_ota_begin(duk_context *ctx)
{
    esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(partition, OTA_SIZE_UNKNOWN, &handle);
    if (err == ESP_OK)
    {
        duk_push_uint(ctx, handle);
        return 1;
    }
    else
    {
        jslog(ERROR, "esp_ota_begin returned error %i", err);
        return -1;
    }
}

duk_ret_t el_ota_write(duk_context *ctx)
{
    duk_size_t size;
    esp_ota_handle_t handle = duk_to_uint32(ctx, 0);
    void *data = duk_get_buffer_data(ctx, 1, &size);

    esp_err_t err = esp_ota_write(handle, data, size);
    if (err == ESP_OK)
    {
        return 0;
    }
    else
    {
        jslog(ERROR, "Error while ota write: %i", err);
        return -1;
    }
}

duk_ret_t el_ota_end(duk_context *ctx)
{
    esp_ota_handle_t handle = duk_to_uint32(ctx, 0);

    esp_err_t err = esp_ota_end(handle);
    if (err == ESP_OK)
    {
        return 0;
    }
    else
    {
        jslog(ERROR, "Error while ota end: %i", err);
        return -1;
    }
}

duk_ret_t el_ota_switch_boot_partition(duk_context *ctx)
{
    esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    esp_err_t err = esp_ota_set_boot_partition(partition);

    if (err == ESP_OK)
    {
        return 0;
    }
    else
    {
        jslog(ERROR, "Error while ota switching boot: %i", err);
        return -1;
    }
}

duk_ret_t el_ota_find_next_modules_partition(duk_context *ctx)
{
    esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    const char *modulesLabel = strcmp(partition->label, "ota_1") == 0 ? "modules_1" : "modules";
    esp_partition_t *modulesPartition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, modulesLabel);
    if (modulesPartition != NULL)
    {
        duk_push_pointer(ctx, modulesPartition);
        return 1;
    }
    else
    {
        jslog(ERROR, "Next modules partition not found.");
        return -1;
    }
}

duk_ret_t el_find_partition(duk_context *ctx)
{
    char *label = duk_to_string(ctx, 0);
    esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, label);
    if (partition != NULL)
    {
        duk_idx_t obj_idx = duk_push_object(ctx);

        duk_push_pointer(ctx, partition);
        duk_put_prop_string(ctx, obj_idx, "_ref");
        duk_push_uint(ctx, partition->size);
        duk_put_prop_string(ctx, obj_idx, "size");

        return 1;
    }
    else
    {
        jslog(ERROR, "Partition with label %s not found.", label);
        return -1;
    }
}

duk_ret_t el_partition_erase(duk_context *ctx)
{
    esp_partition_t *partition = duk_to_pointer(ctx, 0);
    esp_err_t err = esp_partition_erase_range(partition, 0, partition->size);
    if (err == ESP_OK)
    {
        return 0;
    }
    else
    {
        jslog(ERROR, "Error while erasing next modules partition: %i", err);
        return -1;
    }
}

duk_ret_t el_partition_write(duk_context *ctx)
{
    esp_partition_t *partition = duk_to_pointer(ctx, 0);
    size_t offset = duk_to_uint(ctx, 1);

    size_t size;
    void *data = duk_get_buffer_data(ctx, 2, &size);

    esp_err_t err = esp_partition_write(partition, offset, data, size);
    if (err == ESP_OK)
    {
        return 0;
    }
    else
    {
        jslog(ERROR, "Error while erasing next modules partition: %i", err);
        return -1;
    }
}

bool isNativeOtaSupported()
{
    if (esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, "otadata") == NULL)
    {
        return false;
    }
    else
    {
        return true;
    }
}

duk_ret_t el_is_native_ota_supported(duk_context *ctx)
{
    duk_push_boolean(ctx, isNativeOtaSupported());
    return 1;
}

void ota_module_init(duk_context *ctx) {
	    if (isNativeOtaSupported())
    {
        duk_push_c_function(ctx, el_ota_begin, 0 /*nargs*/);
        duk_put_global_string(ctx, "el_ota_begin");

        duk_push_c_function(ctx, el_ota_write, 2 /*nargs*/);
        duk_put_global_string(ctx, "el_ota_write");

        duk_push_c_function(ctx, el_ota_end, 1 /*nargs*/);
        duk_put_global_string(ctx, "el_ota_end");

        duk_push_c_function(ctx, el_ota_switch_boot_partition, 0 /*nargs*/);
        duk_put_global_string(ctx, "el_ota_switch_boot_partition");

        duk_push_c_function(ctx, el_ota_find_next_modules_partition, 0 /*nargs*/);
        duk_put_global_string(ctx, "el_ota_find_next_modules_partition");
    }

    duk_push_c_function(ctx, el_is_native_ota_supported, 0 /*nargs*/);
    duk_put_global_string(ctx, "el_is_native_ota_supported");

    duk_push_c_function(ctx, el_partition_erase, 1 /*nargs*/);
    duk_put_global_string(ctx, "el_partition_erase");

    duk_push_c_function(ctx, el_partition_write, 3 /*nargs*/);
    duk_put_global_string(ctx, "el_partition_write");

    duk_push_c_function(ctx, el_find_partition, 1 /*nargs*/);
    duk_put_global_string(ctx, "el_find_partition");

	
}
