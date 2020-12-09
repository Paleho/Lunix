#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
int main(int argc, char ** argv){
	int fd = open("/dev/lunix0-temp", O_RDONLY);
	if(fd < 0){
		write(2, "open failed\n", 12);
		return 0;
	}

	char  b[10];
	ssize_t i = read(fd, b, 4);
	b[i] = '\0';
	printf("read %d bytes: %s\n", i, b);
	i = read(fd, b, 8);
	b[i] = '\0';
	printf("read %d bytes: %s\n", i, b);
	close(fd);
	return 0;
}
