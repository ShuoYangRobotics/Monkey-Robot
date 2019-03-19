#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "checksum.h"

#define MAX_STRING_SIZE	2048

/*
 * int main( int argc, char *argv[] );
 *
 * The function main() is the entry point of the example program which
 * calculates several CRC values from the contents of files, or data from
 * stdin.
 */

int main( int argc, char *argv[] ) {

	char input_string[MAX_STRING_SIZE];
	unsigned char *ptr;
	unsigned char *dest;
	unsigned char hex_val;
	unsigned char prev_byte;
	uint16_t crc_16_val;
	uint16_t crc_16_modbus_val;
    uint16_t crc_ccitt_0000_val;
	uint16_t crc_ccitt_ffff_val;
    uint16_t crc_ccitt_1d0f_val;    
	int a;
	int ch;
	bool do_ascii;
	bool do_hex;
	FILE *fp;

	do_ascii = false;
	do_hex   = false;

	printf( "\ntstcrc: CRC algorithm sample program\nCopyright (c) 1999-2016 Lammert Bies\n\n" );

	if ( argc < 2 ) {

		printf( "Usage: tst_crc [-a|-x] file1 ...\n\n" );
		printf( "    -a Program asks for ASCII input. Following parameters ignored.\n" );
		printf( "    -x Program asks for hexadecimal input. Following parameters ignored.\n" );
		printf( "       All other parameters are treated like filenames. The CRC values\n" );
		printf( "       for each separate file will be calculated.\n" );

		exit( 0 );
	}

	if ( ! strcmp( argv[1], "-a" )  ||  ! strcmp( argv[1], "-A" ) ) do_ascii = true;
	if ( ! strcmp( argv[1], "-x" )  ||  ! strcmp( argv[1], "-X" ) ) do_hex   = true;

	if ( do_ascii  ||  do_hex ) {

		printf( "Input: " );
		if( fgets( input_string, MAX_STRING_SIZE-1, stdin ) == NULL ){
                   // Print the error
                   perror("Error");
                }
	}

	if ( do_ascii ) {

		ptr = (unsigned char *) input_string;
		while ( *ptr  &&  *ptr != '\r'  &&  *ptr != '\n' ) ptr++;
		*ptr = 0;
	}

	if ( do_hex ) {

		ptr  = (unsigned char *) input_string;
		dest = (unsigned char *) input_string;

		while( *ptr  &&  *ptr != '\r'  &&  *ptr != '\n' ) {

			if ( *ptr >= '0'  &&  *ptr <= '9' ) *dest++ = (unsigned char) ( (*ptr) - '0'      );
			if ( *ptr >= 'A'  &&  *ptr <= 'F' ) *dest++ = (unsigned char) ( (*ptr) - 'A' + 10 );
			if ( *ptr >= 'a'  &&  *ptr <= 'f' ) *dest++ = (unsigned char) ( (*ptr) - 'a' + 10 );

			ptr++;
		}

		* dest    = '\x80';
		*(dest+1) = '\x80';
	}

	a = 1;

	do {

		crc_16_val         = 0x0000;
		crc_16_modbus_val  = 0xffff;
		crc_ccitt_0000_val = 0x0000;
		crc_ccitt_ffff_val = 0xffff;
		crc_ccitt_1d0f_val = 0x1d0f;



		if ( do_ascii ) {

			prev_byte = 0;
			ptr       = (unsigned char *) input_string;

			while ( *ptr ) {

				crc_ccitt_ffff_val = update_crc_ccitt(  crc_ccitt_ffff_val, *ptr            );

				prev_byte = *ptr;
				ptr++;
			}
		}

		else if ( do_hex ) {

			prev_byte = 0;
			ptr       = (unsigned char *) input_string;

			while ( *ptr != '\x80' ) {

				hex_val  = (unsigned char) ( ( * ptr     &  '\x0f' ) << 4 );
				hex_val |= (unsigned char) ( ( *(ptr+1)  &  '\x0f' )      );

				crc_ccitt_ffff_val = update_crc_ccitt(  crc_ccitt_ffff_val, hex_val            );

				prev_byte = hex_val;
				ptr      += 2;
			}

			input_string[0] = 0;
		}

		else {

			prev_byte = 0;
#if defined(_MSC_VER)
			fp = NULL;
			fopen_s( & fp, argv[a], "rb" );
#else
			fp = fopen( argv[a], "rb" );
#endif

			if ( fp != NULL ) {

				while( ( ch=fgetc( fp ) ) != EOF ) {

					crc_ccitt_ffff_val = update_crc_ccitt(  crc_ccitt_ffff_val, (unsigned char) ch            );

					prev_byte = (unsigned char) ch;
				}

				fclose( fp );
			}

			else printf( "%s : cannot open file\n", argv[a] );
		}

		printf( "%s%s%s :CRC-CCITT (0xffff) = 0x%04" PRIX16 "      /  %" PRIu16 "\n"
				, (   do_ascii  ||    do_hex ) ? "\""    : ""
				, ( ! do_ascii  &&  ! do_hex ) ? argv[a] : input_string
				, (   do_ascii  ||    do_hex ) ? "\""    : ""
				, crc_ccitt_ffff_val, crc_ccitt_ffff_val    );

		a++;

	} while ( a < argc );

	return 0;

}  /* main (tstcrc.c) */