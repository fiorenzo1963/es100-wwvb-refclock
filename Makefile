update_shm_one_shot: update_shm_one_shot.c timespec_ops.h
	gcc update_shm_one_shot.c -o update_shm_one_shot

clean:
	rm -f update_shm_one_shot *.o *.out core
