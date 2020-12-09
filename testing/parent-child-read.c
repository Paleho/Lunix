#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/wait.h>
int main(int argc, char ** argv){
	int fd = open("/dev/lunix0-temp", O_RDONLY);
	if(fd < 0){
		write(2, "open failed\n", 12);
		return 0;
	}

    pid_t p = fork();
    if(p < 0) write(2, "fork error\n", 11);
    else if(p == 0){
        char child_buf[8];
        int times = 1;
        while(times <= 10){
            ssize_t i = read(fd, child_buf, 8);
            printf("child_iter %d: read %d bytes: %s\n", times, i, child_buf);
            times++;
        }
    }
    else{
        char parent_buf[8];
        int times = 1;
        while(times <= 10){
            ssize_t i = read(fd, parent_buf, 8);
            printf("parent_iter %d: read %d bytes: %s\n", times, i, parent_buf);
            times++;
        }
        wait(NULL);
    }
	close(fd);
	return 0;
}
