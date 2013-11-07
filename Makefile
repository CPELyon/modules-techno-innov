# Makefile for control

NAME = mod_gpio
LPC = lpc1224
CPU = cortex-m0
ARCH = armv6-m

CROSS_COMPILE = arm-linux-gnueabi-
CC = $(CROSS_COMPILE)gcc

FOPTS = -fmessage-length=0 -ffunction-sections -fdata-sections \
		-static -fno-builtin
CFLAGS = -Wall -O2 -mthumb -mcpu=$(CPU) $(FOPTS)
#CFLAGS = -Wall -O2 -mthumb -march=$(ARCH) $(FOPTS)
LINKOPTS = -nostartfiles -Wl,--gc-sections -Wl,--build-id=none \
		   -Wl,-Map=lpc_map_$(LPC).map -Tlpc_link_$(LPC).ld


.PHONY: all
all: $(NAME)

prog: CFLAGS += -DEEPROM_WRITE
prog: clean all


INCLUDES = include/
OBJDIR = objs

SRC = $(shell find . -name \*.c)
OBJS = ${SRC:%.c=${OBJDIR}/%.o}

$(NAME): $(OBJS)
	@echo "Linking ..."
	@$(CC) $(CFLAGS) $(LINKOPTS) $(OBJS) -o $@ -I$(INCLUDES)
	@echo "Creating image : [32m$@.bin[37m"
	@$(CROSS_COMPILE)objcopy -R .stack -R .bss -O binary $@ $@.bin
	@ls -l $@.bin
	@echo Done.

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
