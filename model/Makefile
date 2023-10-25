# This Makefile generates files for printing SCAD documents
# Copyright (C) 2016  Kenneth Barbour | kenbarbour.com
# License: GNU GPL v3 (or later)
## make all		Generate STL for each SCAD file
## make gcode		Generate gcode for each SCAD file
rover_bottom.stl: rover.scad lib.scadl
rover_body.stl: rover.scad lib.scadl




DIRS = src src/subdir1 src/subdir2
SEARCHC = $(addsuffix /*.c ,$(DIRS))
SEARCHCPP = $(addsuffix /*.cpp ,$(DIRS))
SRCS = $(wildcard $(SEARCHC))
SRCS += $(wildcard $(SEARCHCPP))

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
	$(SCADC) -o $@ $<

#%.gx: %.stl
#	$(SLICER) $< -o $@

clean:
	rm -f *.stl *.gcode
