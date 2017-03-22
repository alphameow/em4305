#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <signal.h>

//#define TRACE_READ
/*****************************************************************************/
#define DEFAULT_SERIAL_PORT		"/dev/ttyUSB0"

#define	EM4305_CMDWRITE_DATA_LEN	11
#define	EM4305_CMDRESP_DATA_LEN		10

#define	BYTES_OF_ID			8 /* 64bits */
#define	BYTES_PER_PAGE	4

/*****************************************************************************/
static int run_stop = 0;
static int serial_fd = -1;

static const unsigned char EM4305_CONF_SING[BYTES_PER_PAGE] = {0x5F, 0x80, 0x01, 0x00};
static const unsigned char EM4305_CONF_DUAL[BYTES_PER_PAGE] = {0x5F, 0x00, 0x02, 0x00};

static const unsigned char EM4305_CMD_READ_PAGE1[] = {0xAA, 0x0A, 0x02, 0x85, 0x01, 0x8C, 0xBB};
static const unsigned char EM4305_CMD_READ_PAGE4[] = {0xAA, 0x0A, 0x02, 0x85, 0x04, 0x89, 0xBB};
static const unsigned char EM4305_CMD_READ_PAGE5[] = {0xAA, 0x0A, 0x02, 0x85, 0x05, 0x88, 0xBB};
static const unsigned char EM4305_CMD_READ_PAGE6[] = {0xAA, 0x0A, 0x02, 0x85, 0x06, 0x8B, 0xBB};
static const unsigned char EM4305_CMD_READ_PAGE7[] = {0xAA, 0x0A, 0x02, 0x85, 0x07, 0x8A, 0xBB};
static const unsigned char EM4305_CMD_READ_PAGE8[] = {0xAA, 0x0A, 0x02, 0x85, 0x08, 0x85, 0xBB};

static const unsigned char EM4305_CMD_WRITE_RESP[] = {0xAA, 0x0A, 0x02, 0x00, 0x80, 0x88, 0xBB};
/*****************************************************************************/
static void usage(void)
{
	printf( 
					"Usage:\n"
					"\n"
					"  -s serial port device\n"
					"    The device that connect to RFID RW board, default /dev/ttyUSB0.\n\n"
					"  -1 decimal number\n"
					"    The first data that write to ID card.\n\n"
					"  -2 decimal custom id\n"
					"    The first custom id that write to ID card.\n\n"
					"  -3 decimal number\n"
					"    The second data that write to ID card.\n\n"
					"  -4 decimal custom id\n"
					"    The second custom id that write to ID card.\n\n"
					);
}

static int open_serial(const char *portname)
{
	struct termios tty;
	
	serial_fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
	if (serial_fd < 0) {
		printf("failed to open %s ! error %d - %s\n", portname, errno, strerror(errno));
		return -1;
	}
	
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd, &tty) != 0) {
  	printf("tcgetattr: error %d - %s\n", errno, strerror(errno));
  	return -1;
  }

  /* B9600, 8N1 */
	cfsetospeed(&tty, (speed_t)B9600);
  cfsetispeed(&tty, (speed_t)B9600);
  tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS; 
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
  	printf("tcsetattr: error %d - %s\n", errno, strerror(errno));
  	return -1;
  }
  
	return 0;
}

static void clean_all(void)
{
	if (serial_fd > 0) {
		close(serial_fd);
		serial_fd = -1;
	}
}

static int em4305_read(const unsigned char *cmd, int cmdlen, unsigned char *data)
{
	unsigned char buf[16];
	int len;
	int fd = serial_fd;

	memset(buf, 0, sizeof(buf));
	write(fd, cmd, cmdlen);
	usleep(200 * 1000);
	len = read(fd, buf, sizeof(buf));
	if (len <= 0)
		return -1;
#ifdef TRACE_READ
	{
		int i;
		printf("R[%d]:",len);
		for (i = 0; i < len; i++)
			printf(" %02X",buf[i]);
		printf("\n");
	}
#endif
	if (len == EM4305_CMDRESP_DATA_LEN) {
		if ((buf[0] == 0xAA) && (buf[1] == 0x0A) && (buf[2] == 0x05) && (buf[3] == 0x00)) {
			memcpy(data, &buf[4], 4);
			return 0;
		}
	}
	return -1;
}

static int em4305_readn(const unsigned char *cmd, int cmdlen, unsigned char *data)
{
	int i;
	for (i = 0; i < 3; i++) {
		if (em4305_read(cmd, cmdlen, data) == 0)
			return 0;
	}
	return -1;
}

static void em4305_makeid(unsigned char cid, unsigned long number, unsigned char *page_first, unsigned char *page_second)
{
	unsigned char ids[BYTES_OF_ID];  
	unsigned char col_cs = 0; /* column checksum */
	unsigned char cs;
	int pos = BYTES_OF_ID * 8 - 1; 
	int i, bit;
	int row_cs_idx_of_arr, row_cs_idx_of_byte;
	unsigned char reverse_bit_arr[16] = {0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15};
	int hi, lo;

	memset(ids, 0, sizeof(ids));
	ids[0] = 0xFF;
	ids[1] = 0x80;
	pos -= 5; /* lowest is 5 bits column checksum */
	for (i = 0; i < (32+8); i++) {
		if (i < 32)
			bit = (number >> i) & 0x1;
		else
			bit = (cid >> (i-32)) & 0x1;
		cs = (((col_cs >> ((i%4)+1)) & 0x1) + bit) % 2;
		if (cs)
			col_cs |= 1 << ((i%4)+1);
		else
			col_cs &= ~(1 << ((i%4)+1));
		if (i%4 == 0)
			pos--; /* row checksum bit */
		ids[pos/8] |= bit << ((63-pos)%8);
		row_cs_idx_of_arr = (pos+(i%4)+1)/8;
		row_cs_idx_of_byte = (63-(pos+(i%4)+1))%8;
		cs = ((((ids[row_cs_idx_of_arr] >> row_cs_idx_of_byte) & 0x1) + bit) % 2);
		if (cs)
			ids[row_cs_idx_of_arr] |= 1 << row_cs_idx_of_byte;
		else
			ids[row_cs_idx_of_arr] &= ~(1 << row_cs_idx_of_byte);
		pos--;
	}
	ids[BYTES_OF_ID - 1] |= (ids[BYTES_OF_ID - 1] & ~0x1F) | col_cs;

	/* reverse bits of byte */
	for (i = 0; i < BYTES_OF_ID; i++) {
		hi = (ids[i] >> 4) & 0xF;
		lo = ids[i] & 0xF;
		ids[i] = (reverse_bit_arr[lo] << 4) | reverse_bit_arr[hi];
	}

	memcpy(page_first, ids, 4);
	memcpy(page_second, ids + 4, 4);
}

static void em4305_makecmd(unsigned char pageidx, const unsigned char *val, unsigned char *cmd)
{
	cmd[0] = 0xAA;
	cmd[1] = 0x0A;
	cmd[2] = 0x06;
	cmd[3] = 0x84;
	cmd[4] = pageidx;
	memcpy(&cmd[5], val, 4);
	cmd[9] = cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[5] ^ cmd[6] ^ cmd[7] ^ cmd[8];
	cmd[10] = 0xBB;
}

static int em4305_write_data(int pageidx, const unsigned char *data)
{
	unsigned char cmd[EM4305_CMDWRITE_DATA_LEN] = {0};
	unsigned char resp[16] = {0};
	int len;

	em4305_makecmd(pageidx, data, cmd);
	write(serial_fd, cmd, sizeof(cmd));
	usleep(300 * 1000);
	len = read(serial_fd, resp, sizeof(resp));
	if ((len == sizeof(EM4305_CMD_WRITE_RESP)) && memcmp(resp, EM4305_CMD_WRITE_RESP, len) == 0)
		return 0;
#ifdef TRACE_READ
	{
		int i;
		printf("R[%d]:",len);
		for (i = 0; i < len; i++)
			printf(" %02X",resp[i]);
		printf("\n");
	}
#endif
	return -1;
}

static void sig_teardown(int sig)
{
	run_stop = 1;
}

int main(int argc, char **argv)
{
	int opt;
	char portname[32] = {0};
	unsigned long data1 = 0, data2 = 0;
	unsigned char cid1 = 0, cid2 = 0;
	int data_mask = 0;
	int read_mode = 0;
	int i, len;
	unsigned char rbuf[32];
	unsigned long loopcnt;
	unsigned char id_page_first[BYTES_PER_PAGE], id_page_second[BYTES_PER_PAGE];
	const unsigned char *conf_page;

	if (argc < 2) {
		usage();
		return -1;
	}
	
	while ((opt = getopt(argc, argv, "s:1:2:3:4:rh")) != -1) {
		switch (opt) {
			case 's':
				strncpy(portname, optarg, sizeof(portname) - 1);
				break;
			case '1':
				data1 = (unsigned long)atol(optarg);
				data_mask |= 0x1;
				break;
			case '2':
				cid1 = (unsigned char)atol(optarg);
				break;
			case '3':
				data2 = (unsigned long)atol(optarg);
				data_mask |= 0x2;
				break;
			case '4':
				cid2 = (unsigned char)atol(optarg);
				break;
			case 'r':
				read_mode = 1;
				break;
			case 'h':
			default:
				usage();
				return -1;
		}
	} /* end while */

	if (!read_mode && !(data_mask & 0x1)) {
		printf("data MUST input!\n");
		return -1;
	}

	if (strlen(portname) == 0) {
		strcpy(portname, DEFAULT_SERIAL_PORT);
	}
	
	printf("open serial port %s ...\n", portname);
	if(open_serial(portname) < 0)
		return -1;

	signal(SIGINT, sig_teardown);
	signal(SIGTERM, sig_teardown);

	printf("RFID probe ...\n");
	loopcnt = 0;
	while (1) {
		if (run_stop) {
			printf("shutdown ...\n");
			break;
		}

		if (em4305_read(EM4305_CMD_READ_PAGE1, sizeof(EM4305_CMD_READ_PAGE1), rbuf) == 0) {
			if (read_mode) {
				printf("UID   : %02X%02X%02X%02X\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
				if (em4305_readn(EM4305_CMD_READ_PAGE4, sizeof(EM4305_CMD_READ_PAGE4), rbuf) == 0)
					printf("PAGE4 : %02X%02X%02X%02X\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
				if (em4305_readn(EM4305_CMD_READ_PAGE5, sizeof(EM4305_CMD_READ_PAGE5), rbuf) == 0)
					printf("PAGE5 : %02X%02X%02X%02X\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
				if (em4305_readn(EM4305_CMD_READ_PAGE6, sizeof(EM4305_CMD_READ_PAGE6), rbuf) == 0)
					printf("PAGE6 : %02X%02X%02X%02X\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
				if (em4305_readn(EM4305_CMD_READ_PAGE7, sizeof(EM4305_CMD_READ_PAGE7), rbuf) == 0)
					printf("PAGE7 : %02X%02X%02X%02X\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
				if (em4305_readn(EM4305_CMD_READ_PAGE8, sizeof(EM4305_CMD_READ_PAGE8), rbuf) == 0)
					printf("PAGE8 : %02X%02X%02X%02X\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
			} else {
				em4305_makeid(cid1, data1, id_page_first, id_page_second);
				printf("Write PAGE5: %02X%02X%02X%02X\n", id_page_first[0], id_page_first[1], id_page_first[2], id_page_first[3]);
				if (em4305_write_data(5, id_page_first) != 0) {
					printf("failed to write PAGE5!\n");
					break;
				}
				printf("Write PAGE6: %02X%02X%02X%02X\n", id_page_second[0], id_page_second[1], id_page_second[2], id_page_second[3]);
				if (em4305_write_data(6, id_page_second) != 0) {
					printf("failed to write PAGE6!\n");
					break;
				}

				conf_page = EM4305_CONF_SING;
				
				if (data_mask & 0x2) {
					em4305_makeid(cid2, data2, id_page_first, id_page_second);
					printf("Write PAGE7: %02X%02X%02X%02X\n", id_page_first[0], id_page_first[1], id_page_first[2], id_page_first[3]);
					if (em4305_write_data(7, id_page_first) != 0) {
						printf("failed to write PAGE7!\n");
						break;
					}
					printf("Write PAGE8: %02X%02X%02X%02X\n", id_page_second[0], id_page_second[1], id_page_second[2], id_page_second[3]);
					if (em4305_write_data(8, id_page_second) != 0) {
						printf("failed to write PAGE8!\n");
						break;
					}
					conf_page = EM4305_CONF_DUAL;
				}
				
				printf("Write CONF: %02X%02X%02X%02X\n", conf_page[0], conf_page[1], conf_page[2], conf_page[3]);
				if (em4305_write_data(4, conf_page) != 0) {
					printf("failed to write CONF!\n");
					break;
				}
			}
			break;
		}

	}
	
exit:
	clean_all();
	return 0;
}

