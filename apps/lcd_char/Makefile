# Makefile for GPIO Demo Module apps

NAME = $(shell basename $(CURDIR))

.PHONY: $(NAME).bin
$(NAME).bin:
	@make -C ../.. --no-print-directory NAME=$(NAME) apps/$(NAME)/$@

clean mrproper:
	@make -C ../.. --no-print-directory $@

