#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp32/spiram.h"
#include "duk_config.h"
#include "duktape.h"
#include "esp32-js-log.h"


bool spiramAvailable = false;

static void my_fatal(void *udata, const char *msg)
{
    (void)udata; /* ignored in this case, silence warning */

    /* Note that 'msg' may be NULL. */
    jslog(ERROR, "*** FATAL ERROR: %s", (msg ? msg : "no message"));
    abort();
}


IRAM_ATTR void *duk_spiram_malloc(void *udata, size_t size)
{
    if (spiramAvailable)
    {
        return heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    }
    else
    {
        return malloc(size);
    }
}
IRAM_ATTR void *spiram_malloc(size_t size)
{
    return duk_spiram_malloc(NULL, size);
}

IRAM_ATTR void *duk_spiram_realloc(void *udata, void *ptr, size_t size)
{
    if (spiramAvailable)
    {
        return heap_caps_realloc(ptr, size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    }
    else
    {
        return realloc(ptr, size);
    }
}
IRAM_ATTR void *spiram_realloc(void *ptr, size_t size)
{
    return duk_spiram_realloc(NULL, ptr, size);
}

IRAM_ATTR void duk_spiram_free(void *udata, void *ptr)
{
    if (spiramAvailable)
    {
        heap_caps_free(ptr);
    }
    else
    {
        free(ptr);
    }
}
IRAM_ATTR void spiram_free(void *ptr)
{
    duk_spiram_free(NULL, ptr);
}

bool spiramAvail()
{
    void *ptr = heap_caps_malloc(1, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (ptr != NULL)
    {
        heap_caps_free(ptr);
        return true;
    }
    return false;
}
duk_context * jsCreateContext() {
	duk_context * ctx;
	spiramAvailable = spiramAvail();
	if(spiramAvailable) {
	    ctx = duk_create_heap(duk_spiram_malloc, duk_spiram_realloc, duk_spiram_free, NULL, my_fatal);
	} else {
		ctx = duk_create_heap_default();
	}
	return ctx;
}
