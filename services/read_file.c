/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2024 Marvell.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* sample test program */
int main(int argc, char *argv[])
{
	FILE *ptr;
	char device[16];
	char ch;
	char buf[17] = {0};

	// Opening file in reading mode
	sprintf(device, "%s", argv[1]);
	ptr = fopen(device, "r");
	if (ptr == NULL) {
	    printf("file can't be opened %s\n", argv[1]);
	    printf("\n please run as below \n	\
		./readfile /dev/mrvl_ptp[0..1]\n");
	    return 0;
	}
	printf("content of this file are\n");
	fgets(buf, 17, ptr);
	printf("\ntimestamp--%s--\n", buf);
	// Closing the file
	fclose(ptr);
	return 0;
}
