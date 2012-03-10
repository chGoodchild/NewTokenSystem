CC      := avr-g++
CXX     := avr-g++
OBJCOPY := avr-objcopy
STRIP   := avr-strip
SIZE    := avr-size
AVRDUDE := avrdude
OBJDUMP := avr-objdump

OPTIMIZATION := -Os
CFLAGS  = $(OPTIMIZATION)

# NOTE: Change this to match your AVR chip type.
ifeq ($(MCU),)
MCU         := attiny4313
endif
PROG_METHOD := stk500v2 # dragon_isp

ifeq ($(PORT),)
PORT        := /dev/ttyUSB0
endif

ifeq ($(PROGRAM),)
$(error "need to define PROGRAM")
endif

# define the default target
SREC   := $(PROGRAM).srec
default: $(SREC)

ifeq ($(SRCS),)
SRCS := $(PROGRAM).c
$(warning "Please define SRCS in your Makefile; taking SRCS as $(SRCS) for now.")
endif

OBJFILES=$(SRCS:.c=.o)
ASSEMBLY=*.s

CFLAGS += -I.

ifeq ($(strip $(MCU)),attiny2313)
CFLAGS         += -D__AVR_ATtiny2313__ -D__ATtiny2313__ -DF_CPU=1000000
AVRDUDE_PARTNO  = t2313
else ifeq ($(strip $(MCU)),attiny4313)
CFLAGS         += -D__AVR_ATtiny4313__ -D__ATtiny4313__ -DF_CPU=1000000
AVRDUDE_PARTNO  = t4313
else ifeq ($(strip $(MCU)),attiny13)
CFLAGS         += -D__AVR_ATtiny13__
AVRDUDE_PARTNO  = t13
else ifeq ($(strip $(MCU)),attiny13a)
CFLAGS         += -D__AVR_ATtiny13A__
AVRDUDE_PARTNO  = t13
else ifeq ($(strip $(MCU)),attiny25)
CFLAGS         += -D__AVR_ATtiny25__
AVRDUDE_PARTNO  = t25
else ifeq ($(strip $(MCU)),at90s2313)
CFLAGS         += -D__AVR_AT90S2313__
AVRDUDE_PARTNO  = 2313
else ifeq ($(strip $(MCU)),atmega16)
CFLAGS         += -D__AVR_ATmega16__
AVRDUDE_PARTNO  = m16
else ifeq ($(strip $(MCU)),atmega1281)
CFLAGS         += -D__AVR_ATmega1281__
AVRDUDE_PARTNO  = m1281
else
$(error "need to handle MCU: $(MCU)")
endif

.SUFFIXES: .c .cpp .s .o

clean:
	rm -f $(SREC) $(PROGRAM) $(PROGRAM)-stripped $(OBJFILES) $(ASSEMBLY)

%.o: %.c
	$(CC) $(CFLAGS) -mmcu=$(MCU) -c $^ -o $@

%.o: %.cpp
	$(CXX) $(CFLAGS) -mmcu=$(MCU) -c $^ -o $@

%.s: %.c
	$(CC) $(CFLAGS) -S $^ -o $@
%.s: %.cpp
	$(CXX) $(CFLAGS) -S $^ -o $@

$(PROGRAM): $(OBJFILES) Makefile ../avr.prog.mk
	$(CC) $(OPTIMIZATION) -mmcu=$(MCU) $(OBJFILES) -o $@
	$(SIZE) $(PROGRAM)

$(PROGRAM)-stripped: $(PROGRAM)
	$(STRIP) $(PROGRAM) -o $@

$(SREC): $(PROGRAM)-stripped
	$(OBJCOPY) -O srec $(PROGRAM)-stripped $@

.PHONY: program terminal
program: $(SREC)
	sudo $(AVRDUDE) -P $(PORT) -c $(PROG_METHOD) -p $(AVRDUDE_PARTNO) -y -e -F
	sudo $(AVRDUDE) -v -P $(PORT) -c $(PROG_METHOD) -p $(AVRDUDE_PARTNO) -D -U flash:w:$(SREC) -F

terminal:
	sudo $(AVRDUDE) -P $(PORT) -c $(PROG_METHOD) -p $(AVRDUDE_PARTNO) -t 