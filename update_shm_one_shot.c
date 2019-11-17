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
 * Manually update SHM segment.
 * Most of the code here has been stolen from ntpd.
 *
 * Copyright (C) 2019, Fio Cattaneo <fio@cattaneo.us>
 *
 * This code is released under dual GPL version 2 and FreeBSD license, at your option.
 *
 */

#include "timespec_ops.h"

#define	LEAP_NOWARNING	0x0	/* normal, no leap second warning */
#define	LEAP_ADDSECOND	0x1	/* last minute of day has 61 seconds */
#define	LEAP_DELSECOND	0x2	/* last minute of day has 59 seconds */
#define	LEAP_NOTINSYNC	0x3	/* overload, clock is free running */

struct shm_time {
	__int32_t		mode;  /* 0 - if valid is set:
					*       use values,
					*       clear valid
					* 1 - if valid is set:
					*       if count before and after read of values is equal,
					*         use values
					*       clear valid
					*/
	volatile __int32_t	count;
	__int32_t		clock_timestamp_sec;
	__int32_t		clock_timestamp_usec;
	__int32_t		receive_timestamp_sec;
	__int32_t		receive_timestamp_usec;
	__int32_t		leap;
	__int32_t		precision;
	__int32_t		nsamples;
	volatile __int32_t    	valid;
	__uint32_t		clock_timestamp_nsec;
	__uint32_t		receive_timestamp_nsec;
	__int32_t		dummy[8];
};

struct shm_time *get_shmseg()
{
	struct shm_time *shm;
	int shmid;

	shmid = shmget(0x4e545030, sizeof (struct shm_time), IPC_CREAT | 0600);
	/* printf("shmid = %d\n", shmid); */
	if (shmid == -1) {
		printf("update_shm_oneshot: error: shmget (unit 0): %s\n", strerror(errno));
		exit(3);
	}
	shm = (struct shm_time *)shmat(shmid, 0, 0);
	/* printf("shm = %p\n", shm); */
	if (shm == (struct shm_time *)-1) {
		printf("update_shm_oneshot: error: shmat (unit 0): %s\n", strerror(errno));
		exit(4);
	}
	return shm;
}

void update_shm(const struct timespec *pps_ts, const struct timespec *local_ts)
{
	struct shm_time *shm = get_shmseg();

	printf("update_shm_oneshot: shm->valid = %d, shm->count = %d, shm->nsamples = %d\n", shm->valid, shm->count, shm->nsamples);
	atomic_thread_fence(memory_order_seq_cst);
	shm->valid = 0;
	atomic_thread_fence(memory_order_seq_cst);
	shm->clock_timestamp_sec = (__int32_t)pps_ts->tv_sec;
	shm->clock_timestamp_usec = (__int32_t)(pps_ts->tv_nsec / 1000);
	shm->clock_timestamp_nsec = (__int32_t)pps_ts->tv_nsec;
	shm->receive_timestamp_sec = (__int32_t)local_ts->tv_sec;
	shm->receive_timestamp_usec = (__uint32_t)(local_ts->tv_nsec / 1000);
	shm->receive_timestamp_nsec = (__uint32_t)local_ts->tv_nsec;
	shm->leap = LEAP_NOWARNING;
	shm->precision = (-4); /* 6.25 msecs */
	shm->count++;
	atomic_thread_fence(memory_order_seq_cst);
	shm->valid = 1;
	atomic_thread_fence(memory_order_seq_cst);
	printf("update_shm_oneshot: shm->valid = %d, shm->count = %d, shm->nsamples = %d\n", shm->valid, shm->count, shm->nsamples);
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

	if (argc != 3) {
		printf("update_shm_one_shot: usage: update_shm_one_shot pps-timestamp local-timestamp\n");
		exit(1);
	}

	cnt = sscanf(argv[1], "%ld.%ld", &pps_timestamp_tv_sec, &pps_timestamp_tv_nsec);
	if (cnt != 2) {
		printf("update_shm_one_shot: error: bad format for pps-timestamp\n");
		exit(2);
	}
	cnt = sscanf(argv[2], "%ld.%ld", &local_timestamp_tv_sec, &local_timestamp_tv_nsec);
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
