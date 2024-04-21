#include "config.h"
#if CONFIG_MAROVER_QUICK_JS

#include <serial-readline.h>
#include "modules.h"
#include "duk_config.h"
#include "duktape.h"
#include <LittleFS.h>
#define TAG  "dukjs"

void ledc_module_init(duk_context *ctx);
duk_context * jsCreateContext();


class DuktapeModule : public Module { public:
    HardwareSerial*out;
    SerialLineReader*reader;
    duk_context *ctx;
    Marover *rover;

    DuktapeModule(HardwareSerial*s) {
        out = s;
        reader = new SerialLineReader(s);
    }

    virtual void setup(Marover *rover) {
        this->rover = rover;
        ctx = jsCreateContext();

        ledc_module_init(ctx);
        bindRover();
        bindFunctions();
        runAutoexec();

        stats("DuktapeJS");
    }

    void bindFunctions() {
        duk_push_c_function(ctx, js_native_print, DUK_VARARGS);
        duk_put_global_string(ctx, "print");
    }

    void bindRover() {
        duk_push_external_buffer(ctx);
        duk_config_buffer(ctx, -1, rover->rc, rover->inChannels*2);
        duk_push_buffer_object(ctx,-1 , 0 , rover->inChannels * 2, DUK_BUFOBJ_UINT16ARRAY);
        duk_put_global_string(ctx, "rc");
        duk_pop(ctx);
    }


static duk_ret_t js_native_print(duk_context *ctx) {
  duk_push_string(ctx, " ");
  duk_insert(ctx, 0);
  duk_join(ctx, duk_get_top(ctx) - 1);
  Serial.printf("%s", duk_to_string(ctx, -1));
  //duk_push_context_dump(ctx);
//  Serial.printf("|%s\n", duk_to_string(ctx, -1));


  return 0;
}

void js_eval (const char *code) {
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

 // duk_pop(ctx);
  duk_pop(ctx);
//  duk_destroy_heap(ctx);
}





    void fileChanged(const char* name) {
        js_eval("print(rc[2]);print(rc[0]);");
    }


    void runAutoexec() {
        auto file = LittleFS.open("/autoexec.js", "r");
        size_t filesize = file.size();
        // Read into temporary Arduino String
        String data = file.readString();
        // Don't forget to clean up!
        file.close();
        js_eval(data.c_str());
    }


    int c = 0;
    virtual void loop() {
        reader->poll();
        if(reader->available()) {
            char text[reader->len()];
            reader->read(text);
            js_eval(text);
        }


        if(rover->rc[2]>0)rover->rc[2]--;
        duk_gc(ctx ,0);
        duk_gc(ctx,0);
        // todo process events
        //js_eval("loop();");

        // todo process events
        duk_get_global_string(ctx,"loop");
        duk_pcall(ctx, 0);
        duk_pop(ctx);
    }


}

;Module* initDuktape(HardwareSerial*s) {return new DuktapeModule(s);}



#endif
