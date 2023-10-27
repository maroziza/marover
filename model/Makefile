# This Makefile generates files for printing SCAD documents
# Copyright (C) 2016  Kenneth Barbour | kenbarbour.com
# License: GNU GPL v3 (or later)
## make all		Generate STL for each SCAD file
## make gcode		Generate gcode for each SCAD file
BRANDING?=false

rover_bottom.stl: rover_bottom.scad rover.scad lib.scadl
rover_body.stl: rover_body.scad rover.scad lib.scadl


DIRS = .
SEARCHC = $(addsuffix /*.scad ,$(DIRS))
SRCS = $(wildcard $(SEARCHC))

## SCAD Compiler
SCADC?=openscad

## Slicing program
SLICER?=flashforge

STL_TARGETS = $(patsubst %.scad,%.stl,$(wildcard *.scad))
#GCODE_TARGETS = $(patsubst %.scad,%.gx,$(wildcard *.scad))

.PHONY: all models gcode clean
all: models 
models: $(STL_TARGETS)
gcode: $(GCODE_TARGETS)
%.stl: %.scad
	$(SCADC) -D BRANDING=$(BRANDING) -o  $@ $<

#%.gx: %.stl
#	$(SLICER) $< -o $@

clean:
	rm -f *.stl *.gcode
