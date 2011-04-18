MCU = attiny2313
F_CPU = 1200000
TARGET = tiny-pong

CFLAGS += -DMANUAL_CONTROL

include avr-tmpl.mk

