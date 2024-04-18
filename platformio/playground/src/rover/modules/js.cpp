#include "config.h"
#if CONFIG_MAROVER_QUICK_JS

#include "modules.h"
#include "duk_config.h"
#include "duktape.h"


class DuktapeModule : public Module { public:
    Stream*out;
    duk_context *ctx;



    DuktapeModule(Stream*s) { out = s; }
    virtual void setup() {
  ctx = duk_create_heap_default();

  duk_push_c_function(ctx, js_native_print, DUK_VARARGS);
  duk_put_global_string(ctx, "print");
stats("DuktapeJS");
    }
int c  = 0;
virtual void loop() {
if(c == 0)   fileChanged("");
if(c++>1000)c=0;
}




static duk_ret_t js_native_print(duk_context *ctx) {
  duk_push_string(ctx, " ");
  duk_insert(ctx, 0);
  duk_join(ctx, duk_get_top(ctx) - 1);
  Serial.printf("%s", duk_to_string(ctx, -1));

  return 0;
}

void js_eval (char *code) {
  Serial.printf("\n>>> Eval Code: %s\n", code);
  duk_push_string(ctx, code);
  duk_int_t rc = duk_peval(ctx);

  if (rc != 0) {
    duk_safe_to_stacktrace(ctx, -1);
  } else {
    duk_safe_to_string(ctx, -1);
  }

  String res = duk_get_string(ctx, -1);

  Serial.printf("\n>>> Reslt: %s\n", res ? res : "null");

  duk_pop(ctx);
//  duk_destroy_heap(ctx);
}





void fileChanged(const char* name) {
    js_eval("print('Hello, world!');");
}





}

;Module* initDuktape(Stream*s) {return new DuktapeModule(s);}



#endif
