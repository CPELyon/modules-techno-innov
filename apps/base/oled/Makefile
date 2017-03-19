# Makefile for "base" apps
# This includes apps for the GPIO Demo Module and for the LPC1224-BO board.

MODULE = $(shell basename $(shell cd .. && pwd && cd -))
NAME = $(shell basename $(CURDIR))

.PHONY: $(NAME).bin
$(NAME).bin:
	@make -C ../../.. --no-print-directory NAME=$(NAME) MODULE=$(MODULE) apps/$(MODULE)/$(NAME)/$@

clean mrproper:
	@make -C ../../.. --no-print-directory $@

