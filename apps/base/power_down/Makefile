# Makefile for "base" apps

MODULE = $(shell basename $(shell cd .. && pwd && cd -))
NAME = $(shell basename $(CURDIR))

.PHONY: $(NAME).bin
$(NAME).bin:
	@make -C ../../.. --no-print-directory NAME=$(NAME) MODULE=$(MODULE) apps/$(MODULE)/$(NAME)/$@

clean mrproper:
	@make -C ../../.. --no-print-directory $@

