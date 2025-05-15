#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>

int main (int argc, char * argv[]){
	char d_on[]="1";
	char d_off[]="0";
	int fd;
	int n = 0;
	int t = 990;
	while (n<500){
		fd= open("/sys/class/gpio/gpio5/value", O_WRONLY);
		write(fd,d_on,1);
		usleep(t);
		write(fd,d_off,1);
		usleep(20000-t);
		close(fd);
		n++;
		}
	return 0;
	}
