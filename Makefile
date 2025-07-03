CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS = -Os -g -Wall -std=c11 -ffreestanding -nostdlib -mthumb -mcpu=cortex-m3 -DSTM32F1
INCLUDES = -I../libopencm3/include
LDFLAGS = -Tlinker.ld -nostdlib
LDLIBS = -L../libopencm3/lib -l:libopencm3_stm32f1.a

SRC = src/main.c startup.s
OBJ = build/main.o build/startup.o
OUT_ELF = build/main.elf
OUT_BIN = build/main.bin

all: $(OUT_BIN)

build:
	mkdir -p build

build/main.o: src/main.c | build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/startup.o: startup.s | build
	$(CC) -c $< -o $@

$(OUT_ELF): $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS) $(LDLIBS)

$(OUT_BIN): $(OUT_ELF)
	$(OBJCOPY) -O binary \
  --only-section=.vectors \
  --only-section=.text \
  --only-section=.rodata \
  $< $@
clean:
	rm -rf build