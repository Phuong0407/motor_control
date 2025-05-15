#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>

int main (int argc, char * argv[]){
	char data[36];
	int fd,fd2;
	while (1){
		fd= open("/sys/class/gpio/gpio17/value", O_RDONLY);
		read(fd,data,1);
		close(fd);
		fd2 = open("/sys/class/gpio/gpio16/value", O_WRONLY);
		write(fd2,data,1);
		close (fd2);
		usleep(100000);
		}
	return 0;
	}
