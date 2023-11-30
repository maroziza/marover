// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"
#include "chasis.h"


extern String WiFiAddr;


httpd_handle_t input_httpd = NULL;



static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    String page = "";
     page += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0\">\n";
 page += "<script>var xhttp = new XMLHttpRequest();</script>";
 page += "<script>function getsend(arg) { xhttp.open('GET', arg +'?' + new Date().getTime(), true); xhttp.send() } </script>";
 page += "<script>function getbattery() { xhttp.open('GET', 'battery', false); xhttp.send(); console.log(xhttp.responseText); document.getElementById(\"battery\").innerHTML=xhttp.responseText } </script>";
 page += " <script type=\"text/javascript\">var timer = setInterval(function(){getbattery();}, 5*1000); </script>";
 //page += "<p align=center><IMG SRC='http://" + WiFiAddr + ":81/stream' style='width:280px;'></p><br/><br/>";
// page += "<p align=center><IMG SRC='http://" + WiFiAddr + ":81/stream' style='width:300px; transform:rotate(180deg);'></p><br/><br/>";
 page += "<p align=center><IMG SRC='http://" + WiFiAddr + ":81/stream' style='width:300px;'></p><br/><br/>";
 
 page += "<p id=\"battery\">Hello World!</p>";
 page += "<p align=center> <button style=width:90px;height:80px onmousedown=getsend('go') onmouseup=getsend('stop') ontouchstart=getsend('go') ontouchend=getsend('stop') ></button> </p>";
 page += "<p align=center>";
 page += "<button style=width:90px;height:80px onmousedown=getsend('left') onmouseup=getsend('stop') ontouchstart=getsend('left') ontouchend=getsend('stop')></button>&nbsp;";
 page += "<button style=width:90px;height:80px onmousedown=getsend('stop') onmouseup=getsend('stop')></button>&nbsp;";
 page += "<button style=width:90px;height:80px onmousedown=getsend('right') onmouseup=getsend('stop') ontouchstart=getsend('right') ontouchend=getsend('stop')></button>";
 page += "</p>";

 page += "<p align=center><button style=width:90px;height:80px onmousedown=getsend('back') onmouseup=getsend('stop') ontouchstart=getsend('back') ontouchend=getsend('stop') ></button></p>";  

 page += "<p align=center>";
 page += "<button style=width:140px;height:40px onmousedown=getsend('led')>LED</button>";
// page += "<button style=width:140px;height:40px onmousedown=getsend('ledon')>LED ON</button>";
// page += "<button style=width:140px;height:40px onmousedown=getsend('ledoff')>LED OFF</button>";
 page += "</p>";
 
    return httpd_resp_send(req, &page[0], strlen(&page[0]));
}

static esp_err_t go_handler(httpd_req_t *req){
    chasis_forward();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, "OK", 2);
}
static esp_err_t back_handler(httpd_req_t *req){
    chasis_back();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t left_handler(httpd_req_t *req){
    chasis_left();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, "OK", 2);
}
static esp_err_t right_handler(httpd_req_t *req){
    chasis_right();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t stop_handler(httpd_req_t *req){
    chasis_stop();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t led_handler(httpd_req_t *req){
    chasis_light(); 
    return httpd_resp_send(req, "OK", 2);
}


static esp_err_t battery_handler(httpd_req_t *req){
    // static int t=0;
    // Serial.println("battery:");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, "Battery: TBD", 12);
}

void startWebControlServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t go_uri = {
        .uri       = "/go",
        .method    = HTTP_GET,
        .handler   = go_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t back_uri = {
        .uri       = "/back",
        .method    = HTTP_GET,
        .handler   = back_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t stop_uri = {
        .uri       = "/stop",
        .method    = HTTP_GET,
        .handler   = stop_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t left_uri = {
        .uri       = "/left",
        .method    = HTTP_GET,
        .handler   = left_handler,
        .user_ctx  = NULL
    };
    
    httpd_uri_t right_uri = {
        .uri       = "/right",
        .method    = HTTP_GET,
        .handler   = right_handler,
        .user_ctx  = NULL
    };
    
    httpd_uri_t led_uri = {
        .uri       = "/led",
        .method    = HTTP_GET,
        .handler   = led_handler,
        .user_ctx  = NULL
    };
    

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t battery_uri = {
        .uri       = "/battery",
        .method    = HTTP_GET,
        .handler   = battery_handler,
        .user_ctx  = NULL
    };

    // config.core_id = 1;
    
    if (httpd_start(&input_httpd, &config) == ESP_OK) {
        esp_err_t ret;
        ret = httpd_register_uri_handler(input_httpd, &battery_uri);
        ret = httpd_register_uri_handler(input_httpd, &index_uri);
        ret = httpd_register_uri_handler(input_httpd, &go_uri); 
        ret = httpd_register_uri_handler(input_httpd, &back_uri); 
        ret = httpd_register_uri_handler(input_httpd, &stop_uri); 
        ret = httpd_register_uri_handler(input_httpd, &left_uri);
        ret = httpd_register_uri_handler(input_httpd, &right_uri);
        ret = httpd_register_uri_handler(input_httpd, &led_uri);        
        Serial.print("Control http://");
        Serial.print(WiFiAddr);
        Serial.printf(":%d/", config.server_port);
        Serial.println();
    }
}


// void stopWebControlServer(){
//     int result = httpd_stop(&input_httpd);
//     Serial.printf("Stopped configuration server: %d \n", result);
// }