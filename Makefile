# Makefile for GPIO Demo Module

NAME = mod_gpio
LPC = lpc1224
CPU = cortex-m0
ARCH = armv6-m

CROSS_COMPILE = arm-linux-gnueabi-
CC = $(CROSS_COMPILE)gcc
FOPTS = -fno-builtin -ffunction-sections -fdata-sections -ffreestanding
CFLAGS = -Wall -O2 -mthumb -mcpu=$(CPU) $(FOPTS)
LINKOPTS = -static -nostartfiles -nostdlib \
		   -Wl,--gc-sections -Wl,--build-id=none \
		   -Wl,-Map=lpc_map_$(LPC).map -Tlpc_link_$(LPC).ld


.PHONY: all
all: $(NAME).bin

prog: CFLAGS += -DEEPROM_WRITE
prog: clean all


INCLUDES = include/
OBJDIR = objs

SRC = $(shell find . -name \*.c)
OBJS = ${SRC:%.c=${OBJDIR}/%.o}

$(NAME).bin: $(NAME)
	@echo "Creating image : [32m$@[39m"
	@$(CROSS_COMPILE)objcopy -R .stack -R .bss -O binary $^ $@
	@ls -l $@
	@echo Done.

$(NAME): $(OBJS)
	@echo "Linking ..."
	@$(CC) $(CFLAGS) $(LINKOPTS) $(OBJS) -o $@ -I$(INCLUDES)

${OBJDIR}/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "-- compiling" $<
	@$(CC) -MMD -MP -MF ${OBJDIR}/$*.d $(CFLAGS) $< -c -o $@ -I$(INCLUDES)

clean:
	find ${OBJDIR} -name "*.o" -exec rm {} \;
	find ${OBJDIR} -name "*.d" -exec rm {} \;
	rm -f *.map

mrproper: clean
	rm -f $(NAME)*
