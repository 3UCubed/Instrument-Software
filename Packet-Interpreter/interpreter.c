#include <stdio.h>
#include <string.h>
#include <stdlib.h>

double intToVoltage(int value, int resolution, int ref)
{
  double voltage;
  if (resolution == 12)
  {
    voltage = (double)(value * ref) / 4095;
  }

  if (resolution == 16)
  {
    voltage = (double)(value * ref) / 65535;
  }
  return voltage;
}

double intToCelsius(int value, int resolution, int ref)
{
  double mVoltage;
  double temperature;
  if (resolution == 12)
  {
    mVoltage = (intToVoltage(value, resolution, ref) * 1000);
    temperature = (mVoltage - 2035) / -4.5;
  }

  if (resolution == 16)
  {
    mVoltage = (intToVoltage(value, resolution, ref) * 1000);
    temperature = (mVoltage - 2035) / -4.5;
  }
  return temperature;
}

int main(int argc, char **argv)
{
  FILE *input = fopen(argv[1], "rb");

  if (input == NULL)
  {
    printf("ERROR OPENING FILE!");
    exit(-1);
  }

  int byte;
  char sync[2];
  int packet = 0;

  char *erpaLabels[8] = {"SYNC", "SEQ", "ADC", "cSWP", "mSWP", "TEMP1", "TEMP2", "ENDmon"};
  int erpaValues[8];
  int erpaIndex = 0;
  int erpaValid = 0;

  char *pmtLabels[3] = {"SYNC", "SEQ", "ADC"};
  int pmtValues[3];
  int pmtIndex = 0;
  int pmtValid = 0;

  char *hkLabels[13] = {"SYNC", "SEQ", "BUSvmon", "BUSimon", "2.5vmon",
                        "3.3vmon", "5vmon", "5vref", "15v",
                        "n3v3", "n5v", "MCU_TEMP", "MCU_VREF"};
  int hkValues[13];
  int hkIndex = 0;
  int hkValid = 0;

  while ((byte = fgetc(input)) != EOF)
  {
    sync[0] = sync[1];
    sync[1] = byte;

    // printf("0x%02X 0x%02X \n", (sync[0] & 0xFF), (sync[1] & 0xFF));

    if ((sync[0] & 0xFF) == 0xAA && (sync[1] & 0xFF) == 0xAA)
    {
      erpaValid = 1;
      erpaIndex = 0;
      packet = 1;
    }
    if ((sync[0] & 0xFF) == 0xBB && (sync[1] & 0xFF) == 0xBB)
    {
      pmtValid = 1;
      pmtIndex = 0;
      packet = 2;
    }
    if ((sync[0] & 0xFF) == 0xCC && (sync[1] & 0xFF) == 0xCC)
    {
      hkValid = 1;
      hkIndex = 0;
      packet = 3;
    }

    if (packet == 1)
    {
      if (erpaValid)
      {
        erpaValues[erpaIndex] = ((sync[0] & 0xFF) << 8) | (sync[1] & 0xFF);
        switch (erpaIndex)
        {
        case 0:
          /* SEQ Bytes; should be 0xAAAA */
          printf("+-------------+\n");
          printf("| \e[31mERPA PACKET\e[m |\n");
          printf("+------------------------------------------------------------+\n");
          printf("| \e[1m%s:\e[m 0x%X", erpaLabels[erpaIndex], erpaValues[erpaIndex]);
          break;
        case 1:
          /* SYNC Bytes; 0-65535 */
          printf(" \e[1m%s:\e[m #%04d", erpaLabels[erpaIndex], erpaValues[erpaIndex]);
          break;
        case 2:
          /* ERPA eADC Bytes; Interpreted as Volts */
          printf(" \e[1m%s:\e[m %.3fv", erpaLabels[erpaIndex], intToVoltage(erpaValues[erpaIndex], 16, 5));
          break;
        case 3:
          /* SWP Commanded */
          printf(" \e[1m%s:\e[m %.1fv", erpaLabels[erpaIndex], intToVoltage(erpaValues[erpaIndex], 12, 3));
          break;
        case 4:
          /* SWP Monitored */
          printf(" \e[1m%s:\e[m %.1fv  |\n", erpaLabels[erpaIndex], intToVoltage(erpaValues[erpaIndex], 12, 3));
          printf("|------------------------------------------------------------|\n");
          break;
        case 5:
          /* TEMP Op-Amp1 */
          printf("| \e[1m%s:\e[m %07.3f°C", erpaLabels[erpaIndex], intToCelsius(erpaValues[erpaIndex], 12, 3));
          break;
        case 6:
          /* TEMP Op-Amp2 */
          printf(" \e[1m%s:\e[m %07.3f°C", erpaLabels[erpaIndex], intToCelsius(erpaValues[erpaIndex], 12, 3));
          break;
        case 7:
          /* ENDMon */
          printf(" \e[1m%s:\e[m %06.3fv          |\n", erpaLabels[erpaIndex], intToVoltage(erpaValues[erpaIndex], 12, 3));
          printf("+------------------------------------------------------------+\n\n");
          break;
        }
        erpaIndex = (erpaIndex + 1) % 8;
      }
      erpaValid = !erpaValid;
    }
    else if (packet == 2)
    {
      if (pmtValid)
      {
        pmtValues[pmtIndex] = ((sync[0] & 0xFF) << 8) | (sync[1] & 0xFF);
        switch (pmtIndex)
        {
        case 0:
          /* SEQ Bytes; should be 0xBBBB */
          printf("+------------+\n");
          printf("| \e[34mPMT PACKET\e[m |\n");
          printf("+-------------------------------------+\n");
          printf("| \e[1m%s:\e[m 0x%X ", pmtLabels[pmtIndex], pmtValues[pmtIndex]);
          break;
        case 1:
          /* SYNC Bytes; 0-65535 */
          printf("\e[1m%s:\e[m #%04d ", pmtLabels[pmtIndex], pmtValues[pmtIndex]);
          break;
        case 2:
          /* PMT eADC Bytes; Interpreted as Volts */
          printf("\e[1m%s:\e[m %.3fv |\n", pmtLabels[pmtIndex], intToVoltage(pmtValues[pmtIndex], 16, 5));
          printf("+-------------------------------------+\n\n");
          break;
        }
        pmtIndex = (pmtIndex + 1) % 3;
      }
      pmtValid = !pmtValid;
    }
    else if (packet == 3)
    {
      if (hkValid)
      {
        hkValues[hkIndex] = ((sync[0] & 0xFF) << 8) | (sync[1] & 0xFF);
        switch (hkIndex)
        {
          char *hkLabels[13] = {"SYNC", "SEQ", "BUSvmon", "BUSimon", "2.5vmon",
                                "3.3vmon", "5vmon", "5vref", "15v",
                                "n3v3", "n5v", "MCU_TEMP", "MCU_VREF"};
        case 0:
          /* SEQ Bytes; should be 0xCCCC */
          printf("+-------------+\n");
          printf("|  \e[32mHK PACKET\e[m  |\n");
          printf("+---------------------------------------------------------+\n");
          printf("| \e[1m%s:\e[m 0x%X ", hkLabels[hkIndex], hkValues[hkIndex]);
          break;
        case 1:
          /* SYNC Bytes; 0-65535 */
          printf("\e[1m%s:\e[m #%04d ", hkLabels[hkIndex], hkValues[hkIndex]);
          break;
        case 3:
          /* BUS_Imon */
          printf("\e[1m%s:\e[m %.3fv |\n", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
          printf("|---------------------------------------------------------|\n");
          break;
        case 4:
          /* 2.5v_mon */
          printf("| \e[1m%s:\e[m %.3fv ", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
          break;
        case 6:
          /* 5v_mon */
          printf("\e[1m%s:\e[m %.3fv           |\n", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
          printf("|---------------------------------------------------------|\n");
          break;
        case 7:
          /* 5vref_mon */
          printf("| \e[1m%s:\e[m %.3fv ", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
          break;
        case 2:
          /* BUS_Vmon */
        case 5:
          /* 3v3_mon */
        case 8:
          /* 15v_mon */
        case 9:
          /* n3v3_mon */
          printf("\e[1m%s:\e[m %.3fv ", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
          break;
        case 10:
          /* n5v_mon */
          printf("\e[1m%s:\e[m %.3fv      |\n", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
          printf("|---------------------------------------------------------|\n");
          break;
        case 11:
          /* MCU_TEMP */
          printf("| \e[1m%s:\e[m %.3fv ", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
          break;
        case 12:
          /* MCU_VREF */
          printf("\e[1m%s:\e[m %.3fv                       |\n", hkLabels[hkIndex], intToVoltage(hkValues[hkIndex], 12, 3));
          printf("+---------------------------------------------------------+\n\n");
          break;
        }
        hkIndex = (hkIndex + 1) % 13;
      }
      hkValid = !hkValid;
    }
  }

  fclose(input);
  return 0;
}
