#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

int t=500;
int b = 1;

void * bouton (void* a){
	char data[36];
	int fd,fd2;
	char prec;
	fd= open("/sys/class/gpio/gpio17/value", O_RDONLY);
	read(fd,&prec,1);
	close(fd);
	while (1){
		fd= open("/sys/class/gpio/gpio17/value", O_RDONLY);
		read(fd,data,1);
		close(fd);
		fd2 = open("/sys/class/gpio/gpio16/value", O_WRONLY);
		write(fd2,data,1);
		close (fd2);
		if (data[0]=='0' && prec=='1'){
			int r = t+b*100;
			if (r>2000 || r<500){
				b=b*(-1);
			}
			t=t+b*100;
		}
		prec=data[0];
		usleep(100000);
		}
	return NULL;
	}
	
void * servo (void * a){
	char d_on[]="1";
	char d_off[]="0";
	int fd;
	while (1){
		fd= open("/sys/class/gpio/gpio5/value", O_WRONLY);
		write(fd,d_on,1);
		usleep(t);
		write(fd,d_off,1);
		usleep(20000-t);
		close(fd);
		}
	return NULL;
}



int main (int argc, char * argv[]){
	pthread_t t1,t2;
	pthread_create(&t1,NULL,bouton,NULL);
	pthread_create(&t2,NULL,servo,NULL);
	pthread_join(t1,NULL);
	pthread_join(t2,NULL);
	return 0;
}
