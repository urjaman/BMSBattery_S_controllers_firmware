#Original Makefile template from Saeid Yazdani (c) 2016
#LICENSE:	GNU-LGPL

.PHONY: all clean

#Compiler
CC = sdcc

#Platform
PLATFORM = stm8

#Product name
PNAME = main

#Directory for helpers
IDIR = StdPeriphLib/inc
SDIR = StdPeriphLib/src

# In case you ever want a different name for the main source file
MAINSRC = $(PNAME).c

# These are the sources that must be compiled to .rel files:
EXTRASRCS = \
	$(SDIR)/stm8s_itc.c \
	$(SDIR)/stm8s_clk.c \
	$(SDIR)/stm8s_iwdg.c \
	$(SDIR)/stm8s_gpio.c \
	$(SDIR)/stm8s_exti.c \
	$(SDIR)/stm8s_uart2.c \
	$(SDIR)/stm8s_tim1.c \
	$(SDIR)/stm8s_tim2.c \
	$(SDIR)/stm8s_adc1.c \
	$(SDIR)/stm8s_flash.c \
	BOdisplay.c \
	ACAcontrollerState.c \
	ACAeeprom.c \
	ACAsetPoint.c \
	ACAcommons.c \
	gpio.c \
	cruise_control.c \
	uart.c \
	adc.c \
	brake.c \
	timers.c \
	pwm.c \
	motor.c \
	PAS.c \
	SPEED.c \
	display.c \
	display_kingmeter.c

HEADERS = BOdisplay.h ACAcommons.h ACAsetPoint.h ACAcontrollerState.h ACAeeprom.h adc.h  brake.h  cruise_control.h  gpio.h  interrupts.h  main.h  motor.h  pwm.h  timers.h  uart.h  PAS.h  SPEED.h  

# The list of .rel files can be derived from the list of their source files
RELS = $(EXTRASRCS:.c=.rel)

INCLUDES = -I$(IDIR) -I.
CFLAGS   = -m$(PLATFORM) --std-c99 --nolospre
ELF_FLAGS = --out-fmt-ihx --debug
LIBS     =
# This just provides the conventional target name "all"; it is optional
# Note: I assume you set PNAME via some means not exhibited in your original file
all: $(PNAME)

# How to build the overall program

$(PNAME): $(MAINSRC) $(RELS)
	$(CC) $(INCLUDES) $(CFLAGS) $(ELF_FLAGS) $(LIBS) $(MAINSRC) $(RELS)

# How to build any .rel file from its corresponding .c file
# GNU would have you use a pattern rule for this, but that's GNU-specific
%.rel: %.c $(HEADERS)
	$(CC) -c $(INCLUDES) $(CFLAGS) $(ELF_FLAGS) $(LIBS) -o$< $<

# Suffixes appearing in suffix rules we care about.
# Necessary because .rel is not one of the standard suffixes.
.SUFFIXES: .c .rel


flash:
	stm8flash -cstlinkv2 -pstm8s105?6 -w$(PNAME).ihx


ifeq ($(OS),Windows_NT)
ENTF = cmd /C del
else
ENTF = rm -f
endif

clean:
	echo "Cleaning files..."
	$(ENTF) *.asm 
	$(ENTF) *.rel
	$(ENTF) *.lk
	$(ENTF) *.lst
	$(ENTF) *.rst
	$(ENTF) *.sym
	$(ENTF) *.cdb
	$(ENTF) *.map
	$(ENTF) *.adb 
	echo "Done."
