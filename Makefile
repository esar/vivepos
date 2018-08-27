PRG            = image
OBJ            = main.o uart.o

MCU_TARGET     = atmega3209
OPTIMIZE       = -O3

F_CPU          = 20000000

DEFS           = -DF_CPU=$(F_CPU)
LIBS           =

# You should not have to change anything below here.

CCDEVS         = /home/stephen/atmel-toolchain/gcc/dev/atmega3209/
CCINC          = /home/stephen/atmel-toolchain/include
CC             = /home/stephen/atmel-toolchain/avr8-gnu-toolchain-linux_x86_64/bin/avr-gcc -B $(CCDEVS) -I $(CCINC) 
OBJCOPY        = /home/stephen/atmel-toolchain/avr8-gnu-toolchain-linux_x86_64/bin/avr-objcopy
OBJDUMP        = /home/stephen/atmel-toolchain/avr8-gnu-toolchain-linux_x86_64/bin/avr-objdump


ASFLAGS        = -mmcu=$(MCU_TARGET) $(DEFS)
CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
LDFLAGS       = -Wl,-Map,$(PRG).map

# Strip unused code
CFLAGS += -ffunction-sections -fno-unroll-loops -fno-inline -fno-jump-tables
LDFLAGS += -Wl,-gc-sections  

#LDFLAGS += --cref -nostartfiles

all: $(PRG).elf lst text hex


flash: hex
	sudo avrdude -p atmega168p -c avrispmkii -P usb -U flash:w:image.hex 


$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf $(OBJ) *.o $(PRG).elf
	rm -rf *.lst *.map *.bin *.lss *.sym *.hex $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -d $< > $@

# Rules for building the .text rom images

text: bin

hex:  $(PRG).hex
bin:  $(PRG).bin
mem:  $(PRG).mem
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -j .rodata -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
	

