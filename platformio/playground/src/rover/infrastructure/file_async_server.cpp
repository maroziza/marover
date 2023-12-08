#include "Arduino.h"
#include "FS.h"
#include <LittleFS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "web_server.h"

#define SPIFFS LittleFS

// Credits : this is a mashup of code from the following repositories, plus OTA firmware update feature
// https://github.com/smford/esp32-asyncwebserver-fileupload-example
// https://randomnerdtutorials.com/esp32-web-server-spiffs-spi-flash-file-system/
// https://github.com/har-in-air/ESP32_ASYNC_WEB_SERVER_SPIFFS_OTA


static File SpiffsFile;
bool IsRebootRequired = false; 




static String server_directory(bool ishtml = false);
static void server_handle_upload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
static void server_handle_SPIFFS_upload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
static String server_string_processor(const String& var);
static void server_handle_OTA_update(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
static int spiffs_chunked_read(uint8_t* buffer, int maxLen);

// list all of the files, if ishtml=true, return html rather than simple text
static String server_directory(bool ishtml) {
  String returnText = "";
  Serial.println("Listing files stored on SPIFFS");
  File root = SPIFFS.open("/");
  File foundfile = root.openNextFile();
  if (ishtml) {
    returnText += "<table align='center'><tr><th align='left'>Name</th><th align='left'>Size</th><th></th><th></th></tr>";
  }



  while (foundfile) {
    bool isdir = foundfile.isDirectory();
    if (ishtml) {
      returnText += "<tr align='left'><td>" + String(foundfile.name()) +(isdir ? "/" :"")  + "</td><td>" + server_ui_size(foundfile.size()) + "</td>";
      if(!isdir){
          returnText += "<td><button class='directory_buttons' onclick=\"directory_button_handler(\'" + String(foundfile.name()) + "\', \'download\')\">Download</button>";
          returnText += "<td><button class='directory_buttons' onclick=\"directory_button_handler(\'" + String(foundfile.name()) + "\', \'delete\')\">Delete</button></tr>";
      } 
    } else {
      returnText += "File: " + String(foundfile.name()) + " Size: " + server_ui_size(foundfile.size()) + "\n";
    }
    foundfile = root.openNextFile();
  }
  if (ishtml) {
    returnText += "</table>";
  }
  root.close();
  foundfile.close();
  return returnText;
}



// replace %SOMETHING%  in webpage with dynamically generated string
static String server_string_processor(const String& var) {
    if (var == "BUILD_TIMESTAMP") {
        return String(__DATE__) + " " + String(__TIME__); 
        }
    else
    if (var == "FREESPIFFS") {
        return server_ui_size((SPIFFS.totalBytes() - SPIFFS.usedBytes()));
        }
    else
    if (var == "USEDSPIFFS") {
        return server_ui_size(SPIFFS.usedBytes());
        }
    else
    if (var == "TOTALSPIFFS") {
        return server_ui_size(SPIFFS.totalBytes());
        }
    else
        return "?";
    }


static int spiffs_chunked_read(uint8_t* buffer, int maxLen) {              
  //Serial.printf("MaxLen = %d\n", maxLen);
  if (!SpiffsFile.available()) {
    SpiffsFile.close();
    return 0;
    }
  else {
    int count = 0;
    while (SpiffsFile.available() && (count < maxLen)) {
      buffer[count] = SpiffsFile.read();
      count++;
      }
    return count;
    }
}




// handles non .bin file uploads to the SPIFFS directory
static void server_handle_SPIFFS_upload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  // make sure authenticated before allowing upload
  if (server_authenticate(request)) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    Serial.println(logmessage);

    if (!index) {
      logmessage = "Upload Start: " + String(filename);
      // open the file on first call and store the file handle in the request object
      request->_tempFile = SPIFFS.open("/" + filename, "w");
      Serial.println(logmessage);
    }

    if (len) {
      // stream the incoming chunk to the opened file
      request->_tempFile.write(data, len);
      logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
      Serial.println(logmessage);
    }

    if (final) {
      logmessage = "Upload Complete: " + String(filename) + ",size: " + String(index + len);
      // close the file handle as the upload is now done
      request->_tempFile.close();
      Serial.println(logmessage);
      request->redirect("/admin/");
    }
  } else {
    Serial.println("Auth: Failed");
    return request->requestAuthentication();
  }
}



// // handles OTA firmware update
// static void server_handle_OTA_update(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
//   // make sure authenticated before allowing upload
//   if (server_authenticate(request)) {
//     String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
//     Serial.println(logmessage);

//     if (!index) {
//       logmessage = "OTA Update Start: " + String(filename);
//       Serial.println(logmessage);
//       if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
//         Update.printError(Serial);
//         }
//     }

//     if (len) {
//      // flashing firmware to ESP
//      if (Update.write(data, len) != len) {
//         Update.printError(Serial);
//         }      
//       logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
//       Serial.println(logmessage);
//     }

//     if (final) {
//      if (Update.end(true)) { //true to set the size to the current progress
//          logmessage = "OTA Complete: " + String(filename) + ",size: " + String(index + len);
//          Serial.println(logmessage);
//           } 
//      else {
//           Update.printError(Serial);
//           }
//       request->redirect("/");
//       }
//   } else {
//     Serial.println("Auth: Failed");
//     return request->requestAuthentication();
//   }
// }

static void server_handle_upload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    // if (filename.endsWith(".bin") ) {
    //   server_handle_OTA_update(request, filename, index, data, len, final);
    //   }
    // else {
      server_handle_SPIFFS_upload(request, filename, index, data, len, final);
    // }
}


void initFileAsyncEndpoints(AsyncWebServer * server){
  // if url isn't found
  server->onNotFound(server_not_found);

  // run handleUpload function when any file is uploaded
  server->onFileUpload(server_handle_upload);

  // visiting this page will cause you to be logged out
  server->on("/logout", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->requestAuthentication();
    request->send(401);
  });

  // presents a "you are now logged out webpage
  server->on("/logged-out", HTTP_GET, [](AsyncWebServerRequest * request) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    Serial.println(logmessage);
    request->send(SPIFFS, "/admin/logout.html", String(), false, server_string_processor);
  });


  server -> serveStatic("/admin/", LittleFS, "/admin/").setDefaultFile("index.html").setTemplateProcessor(server_string_processor);

  // server->on("/admin.html", HTTP_GET, [](AsyncWebServerRequest * request) {
  //   String logmessage = "Client:" + request->client()->remoteIP().toString() + + " " + request->url();
  //   if (server_authenticate(request)) {
  //     logmessage += " Auth: Success";
  //     Serial.println(logmessage);
  //     request->send(SPIFFS, "/admin/index.html", String(), false, server_string_processor);
  //   } else {
  //     logmessage += " Auth: Failed";
  //     Serial.println(logmessage);
  //     return request->requestAuthentication();
  //   }
  // });


  //   // Route to load style.css file
  // server->on("/admin/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
  //   request->send(SPIFFS, "/admin/style.css", "text/css");
  // });

  server->on("/reboot", HTTP_GET, [](AsyncWebServerRequest * request) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    request->send(SPIFFS, "/admin/reboot.html", String(), false, server_string_processor);
    logmessage += " Auth: Success";
    Serial.println(logmessage);
    IsRebootRequired = true;
  });

  server->on("/directory", HTTP_GET, [](AsyncWebServerRequest * request)  {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    if (server_authenticate(request)) {
      logmessage += " Auth: Success";
      Serial.println(logmessage);
      request->send(200, "text/plain", server_directory(true));
    } else {
      logmessage += " Auth: Failed";
      Serial.println(logmessage);
      return request->requestAuthentication();
    }
  });


  server->on("/file", HTTP_GET, [](AsyncWebServerRequest * request) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    if (server_authenticate(request)) {
      logmessage += " Auth: Success";
      Serial.println(logmessage);

      if (request->hasParam("name") && request->hasParam("action")) {
        String fileName = "/";
        const char *fileNameParam = request->getParam("name")->value().c_str();
        const char *fileAction = request->getParam("action")->value().c_str();
        fileName.concat(fileNameParam);

        logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url() + "?name=" + String(fileNameParam) + "&action=" + String(fileAction);


        if (!SPIFFS.exists(fileName)) {
          Serial.println(logmessage + " ERROR: file does not exist");
          request->send(400, "text/plain", "ERROR: file does not exist");
          } 
        else {
          Serial.println(logmessage + " file exists");
          if (strcmp(fileAction, "download") == 0) {
            logmessage += " downloaded";
            SpiffsFile = SPIFFS.open(fileName, "r");
            int sizeBytes = SpiffsFile.size();
            Serial.println("large file, chunked download required");
            AsyncWebServerResponse *response = request->beginResponse("application/octet-stream", sizeBytes, [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
              return spiffs_chunked_read(buffer, maxLen);
              });
            char szBuf[80];
            sprintf(szBuf, "attachment; filename=%s", &fileName[1]);// get past the leading '/'
            response->addHeader("Content-Disposition", szBuf);
            response->addHeader("Connection", "close");
            request->send(response);
            SpiffsFile.close();

            } 
          else 
          if (strcmp(fileAction, "delete") == 0) {
            logmessage += " deleted";
            SPIFFS.remove(fileName);
            request->send(200, "text/plain", "Deleted File: " + String(fileName));
            } 
          else {
            logmessage += " ERROR: invalid action param supplied";
            request->send(400, "text/plain", "ERROR: invalid action param supplied");
            }
          Serial.println(logmessage);
          }
      } 
    else {
      request->send(400, "text/plain", "ERROR: name and action params required");
      }
    } 
  else {
    logmessage += " Auth: Failed";
    Serial.println(logmessage);
    return request->requestAuthentication();
    }
  });
}




