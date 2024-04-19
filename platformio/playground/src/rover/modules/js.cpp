#include "config.h"
#if CONFIG_MAROVER_QUICK_JS

#include "modules.h"
#include "duk_config.h"
#include "duktape.h"
#define TAG  "dukjs"


class DuktapeModule : public Module { public:
    Stream*out;
    duk_context *ctx;

    DuktapeModule(Stream*s) { out = s; }
    Marover *rover;
    virtual void setup(Marover *rover) {
        this->rover = rover;

        ctx = duk_create_heap_default();

        duk_push_external_buffer(ctx);
        duk_config_buffer(ctx, -1, rover->rc, rover->inChannels*2);
        duk_push_buffer_object(ctx,
                       -1 /*index of plain buffer*/,
                       0 /*byte offset*/,
                       rover->inChannels * 2/*byte (!) length */,

                        DUK_BUFOBJ_UINT16ARRAY

                       /*flags and type*/);
        duk_put_global_string(ctx, "rc");
duk_pop(ctx);

        duk_push_c_function(ctx, js_native_print, DUK_VARARGS);
        duk_put_global_string(ctx, "print");

        stats("DuktapeJS");
    }


    int c = 0;
    virtual void loop() {
        //if(c == 0)
    // Serial.printf("%x %d %d\n",rover->rc, rover->rc[0],rover->rc[2]);
     if(rover->rc[2]>0)rover->rc[2]--;
     //   ESP_LOGW(TAG, "rc:%x rc1:%d rc3:%d", rover->rc, rover->rc[0], rover->rc[2]);

        fileChanged("");
        //if(c++ > 100) c=0;
    }




static duk_ret_t js_native_print(duk_context *ctx) {
  duk_push_string(ctx, " ");
  duk_insert(ctx, 0);
  duk_join(ctx, duk_get_top(ctx) - 1);
  Serial.printf("%s", duk_to_string(ctx, -1));
  duk_push_context_dump(ctx);
  Serial.printf("|%s\n", duk_to_string(ctx, -1));


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
    js_eval("print(rc[2]);print(rc[0]);");
}





}

;Module* initDuktape(Stream*s) {return new DuktapeModule(s);}



#endif
