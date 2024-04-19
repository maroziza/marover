/*
 * serial-readline.cpp - Library for buffered serial line reading
 * Created by MSZ, March 3, 2022.
 * Released into the public domain under MIT Licence.
 * Copyright (c) 2022 Zawisza (MSZ98)
*/
#include "soc/soc.h"
#include "serial-readline.h"
int a = 0;

void SerialLineReader::poll() {
    while(hs->available()) {
//      Read single character and save it in the buffer
        char c = hs->read();

        if(buffer_len >= buffer_limit) return;
        if(c < 32) switch(c) {
            case 4: // C-r -restart
                esp_restart();
            case 8:
                if(buffer_len>0)buffer_len--;
                break;
            default:
                hs->printf("KEY:%d %c\n", c,c);

        } else {
            buffer[buffer_len++] = c;
            hs->printf("%c",c);
        }
//      If character saved in the buffer is \n, save this line as independent string, add to the Line Queue and clear the buffer
        if(c == '\n') {
            buffer[buffer_len] = 0;
            char *line = new char[buffer_len];
            strcpy(line, buffer);
            queue.add(line);
            buffer_len = 0;
        }
//      If isr is set and line is ready, execute isr (this automatically disposes one line)
        if(!queue.isEmpty() && isr != NULL) {
            char *line = queue.get();
            isr(line);
            delete line;
        }
    }
}


void SerialLineReader::read(char *line) {
    char *l = queue.get();
    strcpy(line, l);
    delete l;
}
