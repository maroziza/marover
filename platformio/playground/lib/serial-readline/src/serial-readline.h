/*
 * serial-readline.h - Library for buffered serial line reading
 * Created by MSZ, March 3, 2022.
 * Released into the public domain under MIT Licence.
 * Copyright (c) 2022 Zawisza (MSZ98)
*/

#ifndef SERIAL_READLINE_H
#define SERIAL_READLINE_H

#include <Arduino.h>
#include <string.h>

class LineQueue {

private:
	typedef struct {
		char *line;
		void *next;
	} Line;

	Line *first = NULL, *last = NULL;
	int _size = 0;
	
public:
	void add(char* line) {
		Line *l = new Line;
		l->line = line;
		l->next = NULL;
		if(last != NULL) last->next = l;
		last = l;
		if(first == NULL) first = last;
		_size++;
	}
	
	char* get() {
		Line *l = first;
		first = (Line*) l->next;
		if(first == NULL) last = NULL;
		char *line = l->line;
		delete l;
		_size--;
		return line;
	}

	boolean isEmpty() {
		return first == NULL;
	}

	int size() {return _size;}
	int firstLineLength() {return strlen(first->line);}
	
};


class SerialLineReader {

private:
	LineQueue queue;
	HardwareSerial *hs;
	void (*isr)(char*) = NULL;

	int buffer_len = 0;
	int buffer_limit;
	char *buffer;

public:
	SerialLineReader(HardwareSerial *hs, int bufsize, void (*isr)(char*)) {
		initialize(hs, bufsize, isr);
	}
	SerialLineReader(HardwareSerial *hs, void (*isr)(char*)) {
		initialize(hs, 100, isr);
	}
	SerialLineReader(HardwareSerial *hs, int bufsize) {
		initialize(hs, bufsize, NULL);
	}
	SerialLineReader(HardwareSerial *hs) {
		initialize(hs, 100, NULL);
	}
	~SerialLineReader() {
		delete buffer;
	}

	int available() {return queue.size();}
	int len() {return queue.firstLineLength();}
	
	void poll();
	void read(char*);

private:
	void initialize(HardwareSerial *hs, int bufsize, void (*isr)(char*)) {
		this->hs = hs;
		this->isr = isr;
		buffer = new char[bufsize];
		buffer_limit = bufsize;
	}
};


#endif
