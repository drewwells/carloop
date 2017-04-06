flash:
	particle flash tricky-captain

compile: clean
	particle compile electron

serialflash: compile
	particle flash --serial *.bin

logs:
	particle subscribe mine

seriallog:DEVICE=$(shell ls /dev/tty.usb*)
seriallog:
	screen -L $(DEVICE) 112500

clean:
	rm -f *.bin
