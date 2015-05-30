#ifdef WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "hidapi.h"

#define MAX_STR 255

int main(int argc, char* argv[])
{

	short xaccel;
	int res;
	unsigned char buf[65];
	wchar_t wstr[MAX_STR];
	hid_device *handle;
	int i;
	int row;
	char string[25];
	
	

	printf("Input the row number:");
	scanf("%d", &row);

	printf("Input the string:");
	
	scanf("%s", string);
	//fgets(string,100,stdin);

	// Initialize the hidapi library
	res = hid_init();
	
	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(0x4d8, 0x3f, NULL);
	//if (handle==0) {
	//	printf("123");
	//}

	// Read the Manufacturer String
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	wprintf(L"Manufacturer String: %s\n", wstr);
	//printf("Hello");

	// Read the Product String
	res = hid_get_product_string(handle, wstr, MAX_STR);
	wprintf(L"Product String: %s\n", wstr);

	// Read the Serial Number String
	res = hid_get_serial_number_string(handle, wstr, MAX_STR);
	wprintf(L"Serial Number String: (%d) %s\n", wstr[0], wstr);

	// Read Indexed String 1
	res = hid_get_indexed_string(handle, 1, wstr, MAX_STR);
	wprintf(L"Indexed String 1: %s\n", wstr);

	// Toggle LED (cmd 0x80). The first byte is the report number (0x0).
	buf[0] = 0x0;
	buf[1] = 0x80;

	// write number to buf
	buf[2] = row;

	buf[3] = row;
	buf[4] = string[0];
	buf[5] = string[1];
	buf[6] = string[2];
	buf[7] = string[3];
	buf[8] = string[4];
	buf[9] = string[5];
	buf[10] = string[6];
	buf[11] = string[7];
	buf[12] = string[8];
	buf[13] = string[9];
	buf[14] = string[10];
	buf[15] = string[11];
	buf[16] = string[12];
	buf[17] = string[13];
	buf[18] = string[14];
	buf[19] = string[15];
	buf[20] = string[16];
	buf[21] = string[17];
	buf[22] = string[18];
	buf[23] = string[19];
	buf[24] = string[20];
	buf[25] = string[21];
	buf[26] = string[22];
	buf[27] = string[23];
	buf[28] = string[24];
	buf[29] = string[25];

	res = hid_write(handle, buf, 65);

	// Request state (cmd 0x81). The first byte is the report number (0x0).
	buf[0] = 0x0;
	buf[1] = 0x81;
	buf[2] = row;


	buf[3] = row;
	buf[4] = string[0];
	buf[5] = string[1];
	buf[6] = string[2];
	buf[7] = string[3];
	buf[8] = string[4];
	buf[9] = string[5];
	buf[10] = string[6];
	buf[11] = string[7];
	buf[12] = string[8];
	buf[13] = string[9];
	buf[14] = string[10];
	buf[15] = string[11];
	buf[16] = string[12];
	buf[17] = string[13];
	buf[18] = string[14];
	buf[19] = string[15];
	buf[20] = string[16];
	buf[21] = string[17];
	buf[22] = string[18];
	buf[23] = string[19];
	buf[24] = string[20];
	buf[25] = string[21];
	buf[26] = string[22];
	buf[27] = string[23];
	buf[28] = string[24];
	buf[29] = string[25];
	res = hid_write(handle, buf, 65);

	// Read requested state
	res = hid_read(handle, buf, 65);

	// Print out the returned buffer.
	for (i = 0; i < 20; i++)
		printf("buf[%d]: %d\n", i, buf[i]);
		

	// Finalize the hidapi library
	//xaccel= (buf[2] <<8) | (buf[3]);
	//printf("%d\n", xaccel);
	res = hid_exit();

	return 0;
}

