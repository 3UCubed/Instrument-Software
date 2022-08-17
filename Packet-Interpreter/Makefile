uart: uart.o
		gcc uart.o -o uart

uart.o: uart_interpreter.c
		gcc -c -g --std=c99 -Wall uart_interpreter.c -o uart.o

clean:
		rm -f uart uart.o
