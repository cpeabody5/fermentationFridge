PREFIX = arm-none-eabi
CC = $(PREFIX)-gcc
OBJCOPY = $(PREFIX)-objcopy

CFLAGS = -Wall -O0 -mcpu=cortex-m3 -mthumb -nostdlib -nostartfiles -DSTM32F1 \
  -I../libopencm3/include
LDFLAGS = -Tlinker.ld -L../libopencm3/lib -lopencm3_stm32f1

SRC = src/main.c startup.s
OBJ = $(SRC:.c=.o)
OBJ := $(OBJ:.s=.o)

LIBOPENCM3_DIR = ../libopencm3
include $(LIBOPENCM3_DIR)/mk/genlink-config.mk
include $(LIBOPENCM3_DIR)/mk/genlink-rules.mk
include $(LIBOPENCM3_DIR)/mk/gcc-config.mk
include $(LIBOPENCM3_DIR)/mk/gcc-rules.mk


all: build/main.hex

build/main.elf: $(OBJ)
	mkdir -p build
	$(CC) $(CFLAGS) -Tlinker.ld -o $@ $^ -L../libopencm3/lib -lopencm3_stm32f1

build/main.hex: build/main.elf
	$(OBJCOPY) -O ihex \
	--only-section=.vectors \
	--only-section=.text \
	--only-section=.rodata \
	$< $@

build/main.bin: build/main.elf
	$(OBJCOPY) -O binary \
	--only-section=.vectors \
	--only-section=.text \
	--only-section=.rodata \
	$< $@

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -rf build *.o src/*.o
