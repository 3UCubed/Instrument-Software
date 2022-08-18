This is an packet interpreter written in C by Shane Woods. 
This file allows us to take a binary file, read its contents, and interpret them as instrument packets.

The packets should be as follows:
1 ERPA packet every 100ms
1 PMT packet every 125ms
1 Housekeeping packet every 5s

In this folder there is a Makefile that compiles the program, the .c program itself, and a sample binary file with packet information.

To run the program with the sample file:

make uart

./uart screenlog.0

and you should see the packets being printed out to the terminal.
