#for programmer
#PROG=stk500v2
PROG=jtag3isp
#PORT=/dev/ttyUSB0
PORT=usb
BAUD=115200
CPU=m88p
#for compiler
MCU=atmega88pa
F_CPU=16000000
CC=avr-gcc
OBJCOPY=avr-objcopy
#DEPS=color_fun.h common.h eprom_fun.h patterns.h sys_init.h ws2812b.h
BUILD=build

_OBJ=twi-slave-test.o
OBJ = $(patsubst %,$(BUILD)/%,$(_OBJ))

CFLAGS=-std=gnu99 -Wextra -g -O3 -mmcu=${MCU} -DF_CPU=${F_CPU} -I. \
	-funsigned-char -funsigned-bitfields -fshort-enums \
	-Wstrict-prototypes -Wno-missing-field-initializers
#CFLAGS=-std=gnu99 -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I. \
#	-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
#	-Wstrict-prototypes
TARGET=twi-slave-test

# symbolic targets:
all:	$(BUILD)/$(TARGET).hex  $(BUILD)/$(TARGET).eep

$(BUILD)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)	
	
$(BUILD)/$(TARGET).elf: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)
	
$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	rm -f $(TARGET).hex
	$(OBJCOPY) -j .text -j .data -O ihex $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex
	avr-size --format avr --mcu=$(MCU) $(BUILD)/$(TARGET).elf

$(BUILD)/$(TARGET).eep: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@
    
clean:
	rm -f $(BUILD)/*.o $(BUILD)/$(TARGET).elf $(BUILD)/*.bin $(BUILD)/*.hex $(BUILD)/*.eep

#memory writting
flash:
	echo Send to FLASH memory
	avrdude -p $(CPU) -c $(PROG) -P $(PORT) -b $(BAUD) -U flash:w:$(BUILD)/$(TARGET).hex
	
eprom:
	echo Send to EEPROM memory
	avrdude -p $(CPU) -c $(PROG) -P $(PORT) -b $(BAUD) -U eeprom:w:$(BUILD)/$(TARGET).eep

hfuse:
	echo High Fuse writting
	avrdude -p $(CPU) -c $(PROG) -P $(PORT) -b $(BAUD) -U hfuse:w:0xd7:m
	
lfuse:
	echo Low Fuse writting
	avrdude -p $(CPU) -c $(PROG) -P $(PORT) -b $(BAUD) -U lfuse:w:0xe7:m
lock:
	echo Lock Bits writting
	avrdude -p $(CPU) -c $(PROG) -P $(PORT) -b $(BAUD) -U lock:w:0xfc:m
sign:
	echo Read signature bits
	avrdude -p $(CPU) -c $(PROG) -P $(PORT) -b $(BAUD) -U signature:r:output.txt:h
