#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <sys/time.h>
#include <endian.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <errno.h>
#include <stdatomic.h>

/*
 * Manually update SHM segment using mode 1.
 * Most of the code here has been stolen from ntpd.
 *
 * Copyright (C) 2019, Fio Cattaneo <fio@cattaneo.us>
 *
 * This code is released under dual GPL version 2 and FreeBSD license, at your option.
 *
 */

#include "timespec_ops.h"

/*
 * must be > 2 and < 16.
 * do you feel lucky, punk?
 */
#define NTP_UNIT	13

#define	LEAP_NOWARNING	0x0	/* normal, no leap second warning */
#define	LEAP_ADDSECOND	0x1	/* last minute of day has 61 seconds */
#define	LEAP_DELSECOND	0x2	/* last minute of day has 59 seconds */
#define	LEAP_NOTINSYNC	0x3	/* overload, clock is free running */

struct shm_time {
	int		mode;  /* 0 - if valid is set:
				*       use values,
				*       clear valid
				* 1 - if valid is set:
				*       if count before and after read of values is equal,
				*         use values
				*       clear valid
				*/
	volatile int	count;
	time_t		clock_timestamp_sec;
	int		clock_timestamp_usec;
	time_t		receive_timestamp_sec;
	int		receive_timestamp_usec;
	int		leap;
	int		precision;
	int		nsamples;
	volatile int   	valid;
	unsigned int	clock_timestamp_nsec;
	unsigned int	receive_timestamp_nsec;
	int		dummy[8];
};

struct shm_time *get_shmseg()
{
	struct shm_time *shm;
	int shmid;

	shmid = shmget((0x4e545030 | NTP_UNIT), sizeof (struct shm_time), IPC_CREAT | 0666);
	/* printf("shmid = %d\n", shmid); */
	if (shmid == -1) {
		printf("update_shm_one_shot: error: shmget (unit %d): %s\n", strerror(errno), NTP_UNIT);
		exit(3);
	}
	shm = (struct shm_time *)shmat(shmid, 0, 0);
	/* printf("shm = %p\n", shm); */
	if (shm == (struct shm_time *)-1) {
		printf("update_shm_one_shot: error: shmat (unit %d): %s\n", strerror(errno), NTP_UNIT);
		exit(4);
	}
	return shm;
}

void update_shm(const struct timespec *pps_ts, const struct timespec *local_ts)
{
	struct shm_time *shm = get_shmseg();

	printf("update_shm_ones_hot: shm->valid = %d, shm->count = %d, shm->nsamples = %d\n", shm->valid, shm->count, shm->nsamples);
	shm->valid = 0;
	atomic_thread_fence(memory_order_seq_cst);
	shm->mode = 1;
	shm->clock_timestamp_sec = (time_t)pps_ts->tv_sec;
	shm->clock_timestamp_usec = (int)(pps_ts->tv_nsec / 1000);
	shm->clock_timestamp_nsec = (unsigned int)pps_ts->tv_nsec;
	shm->receive_timestamp_sec = (time_t)local_ts->tv_sec;
	shm->receive_timestamp_usec = (int)(local_ts->tv_nsec / 1000);
	shm->receive_timestamp_nsec = (unsigned int)local_ts->tv_nsec;
	shm->leap = LEAP_NOWARNING;
	shm->precision = (-4); /* 6.25 msecs */
	shm->count++;
	atomic_thread_fence(memory_order_seq_cst);
	shm->valid = 1;
	atomic_thread_fence(memory_order_seq_cst);
	printf("update_shm_one_shot: shm->valid = %d, shm->count = %d, shm->nsamples = %d\n", shm->valid, shm->count, shm->nsamples);
}

void show_shm(void)
{
	struct shm_time *shm = get_shmseg();

	printf("update_shm_ones_hot: shm->valid = %d, shm->count = %d, shm->nsamples = %d\n", shm->valid, shm->count, shm->nsamples);
	printf("update_shm_ones_hot: shm->clock_timestamp = %ld.%06ld, shm->receive_timestamp = %ld.%06ld\n",
		shm->clock_timestamp_sec,
		shm->clock_timestamp_usec,
		shm->receive_timestamp_sec,
		shm->receive_timestamp_usec);
}

int main(int argc, char **argv)
{
	struct timespec pps_timestamp, local_timestamp;
	/* timespec types differ from one platform to another */
	long pps_timestamp_tv_sec, pps_timestamp_tv_nsec;
	long local_timestamp_tv_sec, local_timestamp_tv_nsec;
	int cnt;

	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	if (argc != 1 && argc != 3) {
		printf("update_shm_one_shot: usage: update_shm_one_shot [ pps-timestamp local-timestamp ]\n");
		exit(1);
	}

	if (argc == 1) {
		show_shm();
		exit(0);
	}

	cnt = sscanf(argv[1], "%ld.%09ld", &pps_timestamp_tv_sec, &pps_timestamp_tv_nsec);
	if (cnt != 2) {
		printf("update_shm_one_shot: error: bad format for pps-timestamp\n");
		exit(2);
	}
	cnt = sscanf(argv[2], "%ld.%09ld", &local_timestamp_tv_sec, &local_timestamp_tv_nsec);
	if (cnt != 2) {
		printf("update_shm_one_shot: error: bad format for local-timestamp\n");
		exit(3);
	}

	pps_timestamp.tv_sec = pps_timestamp_tv_sec;
	pps_timestamp.tv_nsec = pps_timestamp_tv_nsec;
	local_timestamp.tv_sec = local_timestamp_tv_sec;
	local_timestamp.tv_nsec = local_timestamp_tv_nsec;
	printf("update_shm_one_shot: pps=%ld.%09ld local=%ld.%09ld\n",
		(long)pps_timestamp.tv_sec,
		(long)pps_timestamp.tv_nsec,
		(long)local_timestamp.tv_sec,
		(long)local_timestamp.tv_nsec);

	update_shm(&pps_timestamp, &local_timestamp);
}
