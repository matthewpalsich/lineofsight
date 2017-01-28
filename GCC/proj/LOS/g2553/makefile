SCRIPT=blink
OBJECTS=$(SCRIPT_DIRECTORY)/$(SCRIPT).o


GCC_DIR =  ../../../bin
SUPPORT_FILE_DIRECTORY = ../../../include
HEX_DIRECTORY = ./hex_files
OUT_DIRECTORY = ./out_files
SCRIPT_DIRECTORY = ./scripts

DEVICE  = msp430g2553
CC      = $(GCC_DIR)/msp430-elf-gcc
GDB     = $(GCC_DIR)/msp430-elf-gdb

CFLAGS = -I $(SUPPORT_FILE_DIRECTORY) -mmcu=$(DEVICE) -O2 -g
LFLAGS = -L $(SUPPORT_FILE_DIRECTORY)

hex: all
	$(GCC_DIR)/msp430-elf-objcopy $(OUT_DIRECTORY)/$(SCRIPT).out $(HEX_DIRECTORY)/$(SCRIPT).hex -O ihex

all: ${OBJECTS}
	$(CC) $(CFLAGS) $(LFLAGS) $? -o $(OUT_DIRECTORY)/$(SCRIPT).out

debug: all
	$(GDB) ./$(OUT_DIRECTORY)/$(SCRIPT).out

clean: 
	rm -f ./$(HEX_DIRECTORY)/$(SCRIPT).hex ./$(OUT_DIRECTORY)/$(SCRIPT).out $(OBJECTS)
