This is an packet interpreter written in C by Shane Woods. 
This file allows us to take a binary file, read its contents, and interpret them as instrument packets.

The packets should be as follows:
1 ERPA packet every 100ms
1 PMT packet every 125ms
1 Housekeeping packet every 5s

It will print out like below:

+------------+
| PMT PACKET |
+-------------------------------------+
| SYNC: 0xBBBB SEQ: #0080 ADC: 4.992v |
+-------------------------------------+

+-------------+
| ERPA PACKET |
+--------------------------------------------------+
| SYNC: 0xAAAA SEQ: #0100 ADC: 4.966v SWP: 0.0v    |
|--------------------------------------------------|
| TEMP1: 93.085°C TEMP2: 106.435°C ENDmon: 0.692v  |
+--------------------------------------------------+

+-------------+
|  HK PACKET  |
+---------------------------------------------------------+
| SYNC: 0xCCCC SEQ: #0001 BUSvmon: 1.290v BUSimon: 1.547v |
|---------------------------------------------------------|
| 2.5vmon: 1.530v 3.3vmon: 1.397v 5vmon: 1.667v           |
|---------------------------------------------------------|
| 5vref: 1.478v 15v: 0.795v N3v3: 1.563v N5v: 1.514v      |
+---------------------------------------------------------+

You will notice that each packet has a unique SYNC header that allows us to identify which packets we are reading.

0xAAAA for ERPA
0xBBBB for PMT
0xCCCC for Housekeeping
