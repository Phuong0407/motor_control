#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>
#include <pthread.h>

int main (int argc, char * argv[]){
	char d_on[]="1";
	char d_off[]="0";
	int t=600;
	int fd;
	int fdir;
	int n;
	while (1){
		n=0;
		fd= open("/sys/class/gpio/gpio13/value", O_WRONLY);
		fdir= open("/sys/class/gpio/gpio12/value", O_WRONLY);
		while (n<5000){
			write (fdir,d_on,1);
			write(fd,d_on,1);
			usleep(t);
			write(fd,d_off,1);
			usleep(5000-t);
			n+=1;
		}
		while (n<10000){
			write (fdir,d_off,1);
			write(fd,d_on,1);
			usleep(t);
			write(fd,d_off,1);
			usleep(5000-t);
			n+=1;
		}
	}
	return 0;
}
