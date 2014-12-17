# Makefile for DHT11 example for GPIO Demo Module

NAME = $(shell basename $(CURDIR))

.PHONY: $(NAME).bin
$(NAME).bin:
	@make -C ../.. --no-print-directory NAME=$(NAME) apps/$(NAME)/$@

clean mrproper:
	@make -C ../.. --no-print-directory $@
