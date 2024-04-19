#include "duk_config.h"
#include "duktape.h"
#include <stdio.h>
duk_context*     ctx ;


void js_eval (char *code) {
  printf("\n>>> Eval Code: %s\n", code);
  duk_push_string(ctx, code);
  duk_int_t rc = duk_peval(ctx);

  if (rc != 0) {
    duk_safe_to_stacktrace(ctx, -1);
  } else {
    duk_safe_to_string(ctx, -1);
  }

  char* res = duk_get_string(ctx, -1);

  printf("\n>>> Reslt: %s\n", res ? res : "null");

  duk_pop(ctx);
//  duk_destroy_heap(ctx);
}

static duk_ret_t js_native_print(duk_context *ctx) {
  duk_push_string(ctx, " ");
  duk_insert(ctx, 0);
  duk_join(ctx, duk_get_top(ctx) - 1);
  printf("%s", duk_to_string(ctx, -1));
  duk_push_context_dump(ctx);
  printf("|%s\n", duk_to_string(ctx, -1));

  return 0;
}

#define RC_CHANS 18

int main() {
    int inChannels = RC_CHANS;
    uint16_t rc[RC_CHANS] ;


     ctx = duk_create_heap_default();

        duk_push_external_buffer(ctx);
        duk_config_buffer(ctx, -1, &rc[0], inChannels*2);
        duk_push_buffer_object(ctx,
                       -1 /*index of plain buffer*/,
                       0 /*byte offset*/,
                       inChannels * 2/*byte (!) length */,

                        DUK_BUFOBJ_UINT16ARRAY

                       /*flags and type*/);
        duk_put_global_string(ctx, "rc");
	duk_pop(ctx);

        duk_push_c_function(ctx, js_native_print, DUK_VARARGS);
        duk_put_global_string(ctx, "print");

    js_eval("print(rc[3]);print(rc[1]);");
rc[1]=9000;
    js_eval("print(rc[3]);print(rc[1]);");
rc[3]=4000;
    js_eval("print(rc[3]);print(rc[1]);");

    

}