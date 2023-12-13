// WIFI 
#define CONFIG_MAROVER_WIFI_STATION_ENABLED 1  //Connesct to your AP if 1, otherwise creates one

#define CONFIG_MAROVER_WIFI_STATION_SSID "tofu"
#define CONFIG_MAROVER_WIFI_STATION_PWD "justin2020"

#define CONFIG_MAROVER_WIFI_AP_SSID "esp32-sv"
#define CONFIG_MAROVER_WIFI_AP_PWD "qwer1234"

#define CONFIG_MAROVER_BT_CONNECTION_TIMEOUT 3
#define CONFIG_MAROVER_WIFI_STATION_TIMEOUT 5

//QuickJS engine enabled
#define CONFIG_MAROVER_QUICK_JS 0

//WEB
#define CONFIG_MAROVER_WEB_PORT 80
#define CONFIG_MAROVER_WEB_STREAMING_ASYNC 0



#define CONFIG_MAROVER_AXIS_NUMBER 2
#define CONFIG_MAROVER_CHANNELS_NUMBER 8


#define CHASIS_MODE_DUMMY 1
#define CHASIS_MODE_PWM 2
#define CONFIG_MAROVER_CHASIS_MODE CHASIS_MODE_PWM


