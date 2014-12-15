# Makefile for GPIO Demo Module

TARGET_DIR = apps/$(NAME)

LPC = lpc1224
CPU = cortex-m0
ARCH = armv6-m

CROSS_COMPILE ?= arm-linux-gnueabi-
CC = $(CROSS_COMPILE)gcc
FOPTS = -fno-builtin -ffunction-sections -fdata-sections -ffreestanding
CFLAGS = -Wall -O2 -mthumb -mcpu=$(CPU) $(FOPTS)
LINKOPTS = -static -nostartfiles -nostdlib \
		   -Wl,--gc-sections -Wl,--build-id=none \
		   -Wl,-Map=$(TARGET_DIR)/lpc_map_$(LPC).map -Tlpc_link_$(LPC).ld


APPS = $(subst apps/,,$(wildcard apps/*))

.PHONY: all $(APPS)
all: $(APPS)

INCLUDES = include/
TARGET_INCLUDES = $(TARGET_DIR)/
OBJDIR = objs

SRC = $(wildcard */*.c)
OBJS = ${SRC:%.c=${OBJDIR}/%.o}
DEPS = ${OBJS:%.o=$(OBJDIR)/%.d}

NAME_SRC = $(wildcard $(TARGET_DIR)/*.c)
NAME_OBJS = ${NAME_SRC:%.c=${OBJDIR}/%.o}
NAME_DEPS = ${NAME_OBJS:%.o=$(OBJDIR)/%.d}

-include $(DEPS) $(NAME_DEPS)

.SECONDARY: $(OBJS) $(NAME_OBJS)
.PRECIOUS: %.elf
%.elf: $(OBJS) $(NAME_OBJS)
	@echo "Linking $(NAME) ..."
	@$(CC) $(LINKOPTS) $(OBJS) $(NAME_OBJS) -o $@

%.bin: %.elf
	@echo "Creating image : [32m$@[39m"
	@$(CROSS_COMPILE)objcopy -R .stack -R .bss -O binary $^ $@
	@ls -l $@
	@echo Done.

${OBJDIR}/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "-- compiling" $<
	@$(CC) -MMD -MP -MF ${OBJDIR}/$*.d $(CFLAGS) $< -c -o $@ -I$(INCLUDES) -I$(TARGET_INCLUDES)


$(APPS):
	@make --no-print-directory NAME=$@ apps/$@/$@.bin

all_apps: $(APPS)

clean:
	rm -rf $(OBJDIR)

mrproper: clean
	rm -f apps/*/*.bin apps/*/*.elf apps/*/*.map
