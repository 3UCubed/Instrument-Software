# Instrument-Software
###These are the steps to get packets to print out:
  1. Connect an STM32F051 to your machine with a USB cable. Also plug in a UART cable to your machine.
  2. Plug the yellow cable (RX) from UART to PA9. Plug the orange cable (TX) from UART to PA10. Plug in the black cable (ground) to GND. The ADCs are tied      to the STM32 as followed:

         	PA1 = ADC_IN1, BUS_Vmon: instrument bus voltage monitor
	 
	 	PA2 = ADC_IN2, BUS_Imon: instrument bus current monitor
	 
	 	PA3 = ADC_IN3, 5vref_mon: Accurate 5V for ADC monitor
	 
	 	PA7 = ADC_IN7, SWP_mon: Sweep voltage monitor
	 
	 	PC0 = ADC_IN10, 2v5_mon: power monitor
	 
	 	PC1 = ADC_IN11, 3v3_mon: power monitor
	 
	 	PC2 = ADC_IN12, 5v_mon: power monitor
	 
	 	PC3 = ADC_IN13, n3v3_mon: power monitor
	 
	 	PC4 = ADC_IN14, n5v_mon: power monitor
	 
	 	PC5 = ADC_IN15, 15v_mon: power monitor
	 
	 	temp: (internally connected) = ADC_IN16, VSENSE
	 
	 	Vref: (internally connected) = ADC_IN17, VREFINT
	 
  3. Clone the repository using "git clone https://github.com/3UCubed/Instrument-Software.git"
  4. Open "STM32-Firmware" with the STM32CubeIDE, and run main.c
  5. Navigate to "Packet-Interpreter"
  6. Use "screen -L /dev/cu.usbserial-FT61T5FW 115200" to populate screenlog.0
  7. Use "make uart" and then "./uart screenlog.0" to see the outputted packets
