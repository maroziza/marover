/*
 * QuickJS: i2c linux support library
 *
 * Copyright (c) 2024 Maro Ziza
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "quickjs.h"
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <errno.h>
#include <sys/ioctl.h>
#define countof(x) (sizeof(x) / sizeof((x)[0]))
//#define js_get_errno
static ssize_t js_get_errno(ssize_t ret)
{
    if (ret == -1)
        ret = -errno;
    return ret;
}

static JSValue js_ioctl(JSContext *ctx, JSValueConst this_val, int argc, JSValueConst *argv) {
    int fd,b,c, res;
    if (JS_ToInt32(ctx, &fd, argv[0])) return JS_EXCEPTION;
    if (JS_ToInt32(ctx, &b, argv[1])) return JS_EXCEPTION;
    if (JS_ToInt32(ctx, &c, argv[2])) return JS_EXCEPTION;
    res = js_get_errno(ioctl(fd,b,c));
    return JS_NewInt32(ctx, res);
}

static JSValue js_i2c_read_block(JSContext *ctx, JSValueConst this_val,
                                int argc, JSValueConst *argv)
{
    int fd, arg, addr;

//    uint64_t pos, len;
    size_t size;
    ssize_t ret;
    uint8_t *buf;

    if (JS_ToInt32(ctx, &fd, argv[0])) return JS_EXCEPTION;
    if (JS_ToInt32(ctx, &addr, argv[1])) return JS_EXCEPTION;
    if (JS_ToInt32(ctx, &arg, argv[2])) return JS_EXCEPTION;
    buf = JS_GetArrayBuffer(ctx, &size, argv[3]);
    if (!buf) return JS_EXCEPTION;
    if (size>32)
        return JS_ThrowRangeError(ctx, "i2c max read is 32 bytes, but should be less");


    {
    struct i2c_msg ioctl_msg[2] ;
    struct i2c_rdwr_ioctl_data ioctl_data;
    char cmd[1] = {0xFF & arg};


        /* First message is write internal address */
            ioctl_msg[0].len = 1;
        ioctl_msg[0].addr = addr;
        ioctl_msg[0].buf = cmd;
        ioctl_msg[0].flags = 0;

        /* Second message is read data */
        ioctl_msg[1].len	= 	size;
        ioctl_msg[1].addr	= 	addr;
        ioctl_msg[1].buf	=	buf;
        ioctl_msg[1].flags	=	I2C_M_RD;

        /* Package to i2c message to operation i2c device */
        ioctl_data.nmsgs	=	2;
        ioctl_data.msgs		=	ioctl_msg;
        ret = js_get_errno(ioctl(fd, I2C_RDWR, &ioctl_data));

    }

    return JS_NewInt64(ctx, ret);
}




static const JSCFunctionListEntry js_ioctl_funcs[] = {
    JS_CFUNC_DEF("ioctl", 3, js_ioctl ),
    JS_CFUNC_DEF("i2c_read_block", 4, js_i2c_read_block ),
};

static int js_ioctl_init(JSContext *ctx, JSModuleDef *m)
{
    return JS_SetModuleExportList(ctx, m, js_ioctl_funcs,
                                  countof(js_ioctl_funcs));
}

#ifdef JS_SHARED_LIBRARY
#define JS_INIT_MODULE js_init_module
#else
#define JS_INIT_MODULE js_init_module_ioctl
#endif

JSModuleDef *JS_INIT_MODULE(JSContext *ctx, const char *module_name)
{
    JSModuleDef *m;
    m = JS_NewCModule(ctx, module_name, js_ioctl_init);
    if (!m)
        return NULL;
    JS_AddModuleExportList(ctx, m, js_ioctl_funcs, countof(js_ioctl_funcs));
    return m;
}
