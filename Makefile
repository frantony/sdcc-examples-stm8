SDCC = sdcc

SRC = pie-button.c

# STM8L-DISCOVERY
STM8DEVICE = stm8l152?6

.PHONY: all
all: pie-button.ihx

.PHONY: flash
flash: pie-button.ihx
	stm8flash -c stlinkv2 -p $(STM8DEVICE) -w $<

.PHONY: unlock
unlock:
	stm8flash -u -c stlinkv2 -p $(STM8DEVICE)

%.ihx: %.c
	$(SDCC) -lstm8 -mstm8 --out-fmt-ihx $(CFLAGS) $(LDFLAGS) $<

%.rel: %.c
	$(SDCC) -c $(CFLAGS) $<

%.d: %.c
	@set -e; rm -f $@; \
	$(SDCC) -MD -MP $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

-include $(SRC:%.c=%.d)

.PHONY: clean
clean:
	rm -f *.asm *.cdb *.ihx *.lk *.lst *.map *.rel *.rst *.sym *.d
