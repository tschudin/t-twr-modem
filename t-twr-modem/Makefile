#  Makefile

FQBN=esp32:esp32:esp32s3:CDCOnBoot=cdc

all:
	$(info "use the targets 'compile' and/or 'flash'";)

build/t-twr-modem.ino.bin: t-twr-modem.h util/crc12_b91.h util/fht.h util/adc.h
	arduino-cli compile -b $(FQBN) --libraries lib --build-path ./build t-twr-modem.ino

compile: build/t-twr-modem.ino.bin

flash:
	$(foreach var, $(shell ls /dev/tty.usbmodem*), \
		arduino-cli upload --input-dir build -b $(FQBN) \
		--port $(var);)

clean:
	rm -rf build __pycache__ *~

# eof
