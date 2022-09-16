#include <stdio.h>
#include <string.h>
#include <stdlib.h>

double intToVoltage(int value, int resolution, int ref) {
    double voltage;
    if (resolution == 12) {
        voltage = (double) (value * ref) / 4095;
    }

    if (resolution == 16) {
        voltage = (double) (value * ref) / 65535;
    }
    return voltage;
}

double intToCelsius(int value, int resolution, int ref) {
    double mVoltage;
    double temperature;
    if (resolution == 12) {
        mVoltage = (intToVoltage(value, resolution, ref) * 1000);
        temperature = (mVoltage - 2035) / -4.5;
    }

    if (resolution == 16) {
        mVoltage = (intToVoltage(value, resolution, ref) * 1000);
        temperature = (mVoltage - 2035) / -4.5;
    }
    return temperature;
}

int main(int argc, char **argv) {
    FILE *input = fopen(argv[1], "rb");

    if (input == NULL) {
        printf("ERROR OPENING FILE!");
        exit(-1);
    }

    int byte;
    char sync[2];
    int packet = 0;

    char *erpaLabels[7] = {"SYNC", "SEQ", "ADC", "SWP", "TEMP1", "TEMP2", "ENDmon"};
    int erpaValues[7];
    int erpaIndex = 0;
    int erpaValid = 0;

    char *pmtLabels[3] = {"SYNC", "SEQ", "ADC"};
    int pmtValues[3];
    int pmtIndex = 0;
    int pmtValid = 0;

    char *hkLabels[11] = {"SYNC", "SEQ", "BUSvmon", "BUSimon", "2.5vmon",
                          "3.3vmon", "5vmon", "5vref", "15v",
                          "N3v3", "N5v"};
    int hkValues[11];
    int hkIndex = 0;
    int hkValid = 0;

    while ((byte = fgetc(input)) != EOF) {
        sync[0] = sync[1];
        sync[1] = byte;

        // printf("0x%02X 0x%02X \n", (sync[0] & 0xFF), (sync[1] & 0xFF));

        if ((sync[0] & 0xFF) == 0xAA && (sync[1] & 0xFF) == 0xAA) {
            erpaValid = 1;
            erpaIndex = 0;
            packet = 1;
        }
        if ((sync[0] & 0xFF) == 0xBB && (sync[1] & 0xFF) == 0xBB) {
            pmtValid = 1;
            pmtIndex = 0;
            packet = 2;
        }
        if ((sync[0] & 0xFF) == 0xCC && (sync[1] & 0xFF) == 0xCC) {
            hkValid = 1;
            hkIndex = 0;
            packet = 3;
        }

        if (packet == 1) {
            if (erpaValid) {
                erpaValues[erpaIndex] = ((sync[0] & 0xFF) << 8) | (sync[1] & 0xFF);
                switch (erpaIndex) {
                    case 0: // SYNC
                        printf("+-------------+\n");
                        printf("| \e[31mERPA PACKET\e[m |\n");
                        printf("+--------------------------------------------------+\n");
                        printf("| \e[1m%s:\e[m 0x%X", erpaLabels[erpaIndex], erpaValues[erpaIndex]);
                        break;
                    case 1: // SEQ
                        printf(" \e[1m%s:\e[m #%04d", erpaLabels[erpaIndex], erpaValues[erpaIndex]);
                        break;
                    case 2: // ADC
                        printf(" \e[1m%s:\e[m %.03fv", erpaLabels[erpaIndex],
                               intToVoltage(erpaValues[erpaIndex], 16, 5));
                        break;
                    case 3: // SWP
                        printf(" \e[1m%s:\e[m %.01fv    |\n", erpaLabels[erpaIndex],
                               intToVoltage(erpaValues[erpaIndex], 12, 3));
                        printf("|--------------------------------------------------|\n");
                        break;
                    case 4: // TEMP1
                        printf("| \e[1m%s:\e[m %.03f°C", erpaLabels[erpaIndex],
                               intToCelsius(erpaValues[erpaIndex], 12, 3));
                        break;
                    case 5: // TEMP2
                        printf(" \e[1m%s:\e[m %.03f°C", erpaLabels[erpaIndex],
                               intToCelsius(erpaValues[erpaIndex], 12, 3));
                        break;
                    case 6: // ENDmon
                        printf(" \e[1m%s:\e[m %.03fv  |\n", erpaLabels[erpaIndex],
                               intToVoltage(erpaValues[erpaIndex], 12, 3));
                        printf("+--------------------------------------------------+\n\n");
                        break;
                }
                erpaIndex = (erpaIndex + 1) % 7;
            }
            erpaValid = !erpaValid;
        } else if (packet == 2) {
            if (pmtValid) {
                pmtValues[pmtIndex] = ((sync[0] & 0xFF) << 8) | (sync[1] & 0xFF);
                switch (pmtIndex) {
                    case 0:
                        printf("+------------+\n");
                        printf("| \e[34mPMT PACKET\e[m |\n");
                        printf("+-------------------------------------+\n");
                        printf("| \e[1m%s:\e[m 0x%X ", pmtLabels[pmtIndex], pmtValues[pmtIndex]);
                        break;
                    case 1:
                        printf("\e[1m%s:\e[m #%04d ", pmtLabels[pmtIndex], pmtValues[pmtIndex]);
                        break;
                    case 2:
                        printf("\e[1m%s:\e[m %.03fv |\n", pmtLabels[pmtIndex],
                               intToVoltage(pmtValues[pmtIndex], 16, 5));
                        printf("+-------------------------------------+\n\n");
                        break;
                }
                pmtIndex = (pmtIndex + 1) % 3;
            }
            pmtValid = !pmtValid;
        } else if (packet == 3) {
            if (hkValid) {
                hkValues[hkIndex] = ((sync[0] & 0xFF) << 8) | (sync[1] & 0xFF);
                switch (hkIndex) {
                    case 0:
                        printf("+-------------+\n");
                        printf("|  \e[32mHK PACKET\e[m  |\n");
                        printf("+---------------------------------------------------------+\n");
                        printf("| \e[1m%s:\e[m 0x%X ", hkLabels[hkIndex], hkValues[hkIndex]);
                        break;
                    case 1:
                        printf("\e[1m%s:\e[m #%04d ", hkLabels[hkIndex], hkValues[hkIndex]);
                        break;
                    case 3:
                        printf("\e[1m%s:\e[m %.03fv |\n", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
                        printf("|---------------------------------------------------------|\n");
                        break;
                    case 4:
                        printf("| \e[1m%s:\e[m %.03fv ", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
                        break;
                    case 6:
                        printf("\e[1m%s:\e[m %.03fv           |\n", hkLabels[hkIndex],
                               intToVoltage(hkValues[hkIndex], 12, 3));
                        printf("|---------------------------------------------------------|\n");
                        break;
                    case 7:
                        printf("| \e[1m%s:\e[m %.03fv ", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
                        break;
                    case 2:
                    case 5:
                    case 8:
                    case 9:
                        printf("\e[1m%s:\e[m %.03fv ", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
                        break;
                    case 10:
                        printf("\e[1m%s:\e[m %.03fv      |\n", hkLabels[hkIndex],
                               intToVoltage(hkValues[hkIndex], 12, 3));
                        printf("+---------------------------------------------------------+\n\n");
                        break;
                }
                hkIndex = (hkIndex + 1) % 11;
            }
            hkValid = !hkValid;
        }
    }

    fclose(input);
    return 0;
}
