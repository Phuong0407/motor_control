#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

void * bouton (void* a){
	char data[36];
	int fd1,fd4;
	char prec, truc;
	char d_on[]="1";
	char d_off[]="0";
	fd1= open("/sys/class/gpio/gpio17/value", O_RDONLY);
	read(fd1,&prec,1);
	close(fd1);
	while (1){
		fd1= open("/sys/class/gpio/gpio17/value", O_RDONLY);
		read(fd1,data,1);
		close(fd1);
		if (data[0]=='0' && prec=='1'){
			fd4=open("/sys/class/gpio/gpio18/value", O_RDWR);
			read(fd1,&truc,1);
			if (truc=='0'){
				write(fd4,d_on,1);
			}
			else{
				write(fd4,d_off,1);
			}
			close(fd4);
		}
		usleep(100);
		prec=data[0];
		}
		return NULL;
	}
		
void * bouton2 (void* a){
	char data[36];
	int fd1,fd4;
	char prec, truc;
	char d_on[]="1";
	char d_off[]="0";
	fd1= open("/sys/class/gpio/gpio19/value", O_RDONLY);
	read(fd1,&prec,1);
	close(fd1);
	while (1){
		fd1= open("/sys/class/gpio/gpio19/value", O_RDONLY);
		read(fd1,data,1);
		close(fd1);
		if (data[0]=='0' && prec=='1'){
			fd4=open("/sys/class/gpio/gpio16/value", O_RDWR);
			read(fd1,&truc,1);
			if (truc=='0'){
				write(fd4,d_on,1);
			}
			else{
				write(fd4,d_off,1);
			}
			close(fd4);
		}
		usleep(100);
		prec=data[0];
		}
		return NULL;
	}
		

int main (int argc, char * argv[]){
	pthread_t t1,t2;
	pthread_create(&t1,NULL,bouton,NULL);
	pthread_create(&t2,NULL,bouton2,NULL);
	pthread_join(t1,NULL);
	pthread_join(t2,NULL);
	return 0;
}
