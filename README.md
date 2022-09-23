# Instrument-Software
### These are the steps to get packets to print out:
  1. Connect an STM32F051 to your machine with a USB cable. Also plug in a UART cable to your machine.
  2. Plug the yellow cable (RX) from UART to PA9. Plug the orange cable (TX) from UART to PA10. Plug in the black cable (ground) to GND. The ADCs are tied      to the STM32 as followed:
		- PA0 = ADC_IN0, END_mon: entrance/collimator monitor
		- PA1 = ADC_IN1, BUS_Vmon: instrument bus voltage monitor
		- PA2 = ADC_IN2, BUS_Imon: instrument bus current monitor
		- PA3 = ADC_IN3, 5vref_mon: Accurate 5V for ADC monitor
		- PB0 = ADC_IN8, TMP 1: Sweep temperature
		- PB1 = ADC_IN9, TMP 2: feedbacks
	 	- PA7 = ADC_IN7, SWP_mon: Sweep voltage monitor
	 	- PC0 = ADC_IN10, 2v5_mon: power monitor
	 	- PC1 = ADC_IN11, 3v3_mon: power monitor
	 	- PC2 = ADC_IN12, 5v_mon: power monitor
	 	- PC3 = ADC_IN13, n3v3_mon: power monitor
	 	- PC4 = ADC_IN14, n5v_mon: power monitor
	 	- PC5 = ADC_IN15, 15v_mon: power monitor
	 	- temp: (internally connected) = ADC_IN16, VSENSE
	 	- Vref: (internally connected) = ADC_IN17, VREFINT
  3. Clone the repository using `git clone https://github.com/3UCubed/Instrument-Software.git`
  4. Open "Firmware" with the STM32CubeIDE, and run main.c.
  5. In Terminal, navigate to "Packet-Interpreter".
  6. Use `rm screenlog.0` to remove screenlog.0, so a new one can replace it.
  7. Use `screen -L /dev/cu.usbserial-FT61T5FW 115200` to populate screenlog.0. 

  	- If you would like to put it in Stop Mode, type "s" in the terminal. To then wake it, type "£" in the terminal.
  8. Let it run for a few seconds, and use "control" + "a" + "\\" to exit.
  9. Use `make` and then `./interpreter screenlog.0` to see the outputted packets.

The output should appear like this:

![Screen Shot 2022-08-18 at 4 37 37 PM](https://user-images.githubusercontent.com/94400363/185490226-4144273a-c581-47ca-bd33-58f767a845b1.png)
