/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * hlcand.c - userspace daemon for serial line CAN interface driver HLCAN
 *
 * Copyright (c) 2020 Alexander Mohr <hlcan@mohr.io>
 *
 * Based on slcand.c
 * Copyright (c) 2009 Robert Haddon <robert.haddon@verari.com>
 * Copyright (c) 2009 Verari Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>
#include <pwd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <asm/termbits.h> /* struct termios2 */
#include <linux/tty.h>
#include <linux/sockios.h>
#include <linux/serial.h>
#include <stdarg.h>

#include "module/hlcan.h"

/* Change this to whatever your daemon is called */
#define DAEMON_NAME "hlcand"

/* Change this to the user under which to run */
#define RUN_AS_USER "root"

/* The length of ttypath buffer */
#define TTYPATH_LENGTH	256

/* UART flow control types */
#define FLOW_NONE 0
#define FLOW_HW 1
#define FLOW_SW 2

#define DEFAULT_UART_SPEED 2000000

static void fake_syslog(int priority, const char *format, ...)
{
	va_list ap;

	printf("[%d] ", priority);
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	printf("\n");
}

typedef void (*syslog_t)(int priority, const char *format, ...);
static syslog_t syslogger = syslog;

void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: %s [options] <tty> [canif-name]\n\n", prg);
	fprintf(stderr, "Options: -l         (set transciever to listen mode)\n");
	fprintf(stderr, "         -s <speed> (set CAN speed in bits per second)\n");
	fprintf(stderr, "         -S <speed> (set UART speed in baud)\n");
	fprintf(stderr, "         -e         (set interface to extended id mode)\n");
	fprintf(stderr, "         -F         (stay in foreground; no daemonize)\n");
	fprintf(stderr, "         -m <mode>  (0: normal (default), 1: loopback, 2:silent, 3: loopback silent)\n");
	fprintf(stderr, "         -h         (show this help page)\n");
	fprintf(stderr, "\nExamples:\n");
	fprintf(stderr, "hlcand -m 2 -s 500000 /dev/ttyUSB0\n");
	fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

static int slcand_running;
static int exit_code;
static char ttypath[TTYPATH_LENGTH];

static void child_handler(int signum)
{
	switch (signum) {

	case SIGUSR1:
		/* exit parent */
		exit(EXIT_SUCCESS);
		break;
	case SIGALRM:
	case SIGCHLD:
		syslogger(LOG_NOTICE, "received signal %i on %s", signum, ttypath);
		exit_code = EXIT_FAILURE;
		slcand_running = 0;
		break;
	case SIGINT:
	case SIGTERM:
		syslogger(LOG_NOTICE, "received signal %i on %s", signum, ttypath);
		exit_code = EXIT_SUCCESS;
		slcand_running = 0;
		break;
	}
}

static unsigned char hlcan_create_crc(unsigned char* data)
{
	unsigned char i, checksum = 0;

	for (i = HLCAN_CFG_CRC_IDX;
			i < HLCAN_CFG_PACKAGE_LEN - HLCAN_CFG_CRC_IDX - 1;
			++i) {
		checksum += *(data + i);
	}

	return checksum & 0xff;
}

static int command_settings(HLCAN_SPEED speed,
			    HLCAN_MODE mode,
			    HLCAN_FRAME_TYPE frame,
			    int fd)
{
	int cmd_frame_len;
	unsigned char cmd_frame[HLCAN_CFG_PACKAGE_LEN];

	cmd_frame_len = 0;
	cmd_frame[cmd_frame_len++] = HLCAN_PACKET_START;
	cmd_frame[cmd_frame_len++] = HLCAN_CFG_PACKAGE_TYPE;
	cmd_frame[cmd_frame_len++] = 0x12;
	cmd_frame[cmd_frame_len++] = speed;
	cmd_frame[cmd_frame_len++] = frame;
	cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
	cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
	cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
	cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
	cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
	cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
	cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
	cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
	cmd_frame[cmd_frame_len++] = mode;
	cmd_frame[cmd_frame_len++] = 0x01; // ?
	cmd_frame[cmd_frame_len++] = 0;
	cmd_frame[cmd_frame_len++] = 0;
	cmd_frame[cmd_frame_len++] = 0;
	cmd_frame[cmd_frame_len++] = 0;
	cmd_frame[cmd_frame_len++] = hlcan_create_crc(cmd_frame);

	if (write(fd, cmd_frame, HLCAN_CFG_PACKAGE_LEN) < 0) {
		syslogger(LOG_ERR, "write() failed: %s", strerror(errno));
		return -1;
	}

	return 0;
}

static HLCAN_SPEED HLCAN_int_to_speed(const int speed)
{
	switch (speed) {
	case 1000000:
		return HLCAN_SPEED_1000000;
	case 800000:
		return HLCAN_SPEED_800000;
	case 500000:
		return HLCAN_SPEED_500000;
	case 400000:
		return HLCAN_SPEED_400000;
	case 250000:
		return HLCAN_SPEED_250000;
	case 200000:
		return HLCAN_SPEED_200000;
	case 125000:
		return HLCAN_SPEED_125000;
	case 100000:
		return HLCAN_SPEED_100000;
	case 50000:
		return HLCAN_SPEED_50000;
	case 20000:
		return HLCAN_SPEED_20000;
	case 10000:
		return HLCAN_SPEED_10000;
	case 5000:
		return HLCAN_SPEED_5000;
	default:
		return HLCAN_SPEED_INVALID;
	}
}

int main(int argc, char *argv[])
{
	const char *devprefix = "/dev/";
	char *uart_speed_str = NULL;
	static struct ifreq ifr;
	struct termios2 tios;
	char *tty = NULL;
	char *name = NULL;
	char *pch;
	int fd, opt;

	long int uart_speed = DEFAULT_UART_SPEED;
	int run_as_daemon = 1;
	int ldisc = N_HLCAN;

	HLCAN_MODE mode = HLCAN_MODE_NORMAL;
	HLCAN_SPEED speed = HLCAN_SPEED_500000;
	HLCAN_FRAME_TYPE type = HLCAN_FRAME_STANDARD;

	ttypath[0] = '\0';

	while ((opt = getopt(argc, argv, "es:S:m:?hF")) != -1) {
		switch (opt) {
		case 'e':
			type = HLCAN_FRAME_EXTENDED;
			break;
		case 'm':
			errno = 0;
			mode = atoi(optarg);
			if (errno ||
				mode > HLCAN_MODE_LOOPBACK_SILENT || mode < 0)
				print_usage(argv[0]);
			break;
		case 's':
			errno = 0;
			speed = atoi(optarg);
			if (errno)
				print_usage(argv[0]);
			speed = HLCAN_int_to_speed(speed);
			if (speed == HLCAN_SPEED_INVALID)
				print_usage(argv[0]);
			break;
		case 'S':
			uart_speed_str = optarg;
			errno = 0;
			uart_speed = strtol(uart_speed_str, NULL, 10);
			if (errno)
				print_usage(argv[0]);
			break;
		case 'F':
			run_as_daemon = 0;
			break;
		case 'h':
		case '?':
		default:
			print_usage(argv[0]);
			break;
		}
	}

	if (!run_as_daemon)
		syslogger = fake_syslog;

	/* Initialize the logging interface */
	openlog(DAEMON_NAME, LOG_PID, LOG_LOCAL5);

	/* Parse serial device name and optional can interface name */
	tty = argv[optind];
	if (NULL == tty)
		print_usage(argv[0]);

	name = argv[optind + 1];
	if (name && (strlen(name) > sizeof(ifr.ifr_newname) - 1))
		print_usage(argv[0]);

	/* Prepare the tty device name string */
	pch = strstr(tty, devprefix);
	if (pch != tty)
		snprintf(ttypath, TTYPATH_LENGTH, "%s%s", devprefix, tty);
	else
		snprintf(ttypath, TTYPATH_LENGTH, "%s", tty);

	syslogger(LOG_INFO, "starting on TTY device %s", ttypath);

	fd = open(ttypath, O_RDWR | O_NONBLOCK | O_NOCTTY);
	if (fd < 0) {
		syslogger(LOG_NOTICE, "failed to open TTY device %s\n", ttypath);
		perror(ttypath);
		exit(EXIT_FAILURE);
	}

	if (ioctl(fd, TCGETS2, &tios) < 0) {
		syslogger(LOG_NOTICE, "ioctl() failed: %s\n", strerror(errno));
		close(fd);
		exit(EXIT_FAILURE);
	}

	tios.c_cflag &= ~CBAUD;
	tios.c_cflag = BOTHER | CS8 | CSTOPB;
	tios.c_iflag = IGNPAR;
	tios.c_oflag = 0;
	tios.c_lflag = 0;
	tios.c_ispeed = (speed_t) uart_speed;
	tios.c_ospeed = (speed_t) uart_speed;

	// Because of a recent change in linux - https://patchwork.kernel.org/patch/9589541/
	// we need to set low latency flag to get proper receive latency
	struct serial_struct snew;
	ioctl (fd, TIOCGSERIAL, &snew);
	snew.flags |= ASYNC_LOW_LATENCY;
	ioctl (fd, TIOCSSERIAL, &snew);

	if (ioctl(fd, TCSETS2, &tios) < 0) {
		syslogger(LOG_NOTICE, "ioctl() failed: %s\n", strerror(errno));
		close(fd);
		exit(EXIT_FAILURE);
	}

	if (command_settings(speed, mode, type, fd) < 0){
		close(fd);
        	exit(EXIT_FAILURE);
	}

	/* set hlcan like discipline on given tty */
	if (ioctl(fd, TIOCSETD, &ldisc) < 0) {
		perror("ioctl TIOCSETD");
		exit(EXIT_FAILURE);
	}
	
	/* retrieve the name of the created CAN netdevice */
	if (ioctl(fd, SIOCGIFNAME, ifr.ifr_name) < 0) {
		perror("ioctl SIOCGIFNAME");
		exit(EXIT_FAILURE);
	}

	
	syslogger(LOG_NOTICE, "attached TTY %s to netdevice %s\n", ttypath, ifr.ifr_name);

	/* try to rename the created netdevice */
	if (name) {
		int s = socket(PF_INET, SOCK_DGRAM, 0);

		if (s < 0)
			perror("socket for interface rename");
		else {
			/* current slcan%d name is still in ifr.ifr_name */
			memset (ifr.ifr_newname, 0, sizeof(ifr.ifr_newname));
			strncpy (ifr.ifr_newname, name, sizeof(ifr.ifr_newname) - 1);

			if (ioctl(s, SIOCSIFNAME, &ifr) < 0) {
				syslogger(LOG_NOTICE, "netdevice rename to %s failed\n", name);
				perror("ioctl SIOCSIFNAME rename");
				exit(EXIT_FAILURE);
			} else
				syslogger(LOG_NOTICE, "netdevice renamed to %s\n", name);

			close(s);
		}	
	}
	
	/* Daemonize */
	if (run_as_daemon) {
		if (daemon(0, 0)) {
			syslogger(LOG_ERR, "failed to daemonize");
			exit(EXIT_FAILURE);
		}
	} else {
		/* Trap signals that we expect to receive */
		signal(SIGINT, child_handler);
		signal(SIGTERM, child_handler);
	}

	slcand_running = 1;

	/* The Big Loop, wait 1 second */
	while (slcand_running)
		sleep(1);

	/* Reset line discipline */
	syslogger(LOG_INFO, "stopping on TTY device %s", ttypath);
	ldisc = N_TTY;
	if (ioctl(fd, TIOCSETD, &ldisc) < 0) {
		perror("ioctl TIOCSETD");
		exit(EXIT_FAILURE);
	}

	/* Finish up */
	syslogger(LOG_NOTICE, "terminated on %s", ttypath);
	closelog();
	return exit_code;
}
