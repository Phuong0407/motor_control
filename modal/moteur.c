#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>
#include <pthread.h>
int a = 0;

void * moteur (void* b){
	char data[36];
	char data2[36];
	int fd,fd2;
	char prec6='1';
	int sens = 1;
	while (1){
		fd= open("/sys/class/gpio/gpio26/value", O_RDONLY);
		fd2 = open("/sys/class/gpio/gpio27/value", O_RDONLY);
		read(fd,data,1);
		if (data[0]=='1' && prec6=='0'){
			read(fd2,data2,1);
			if (data2[0]=='1'){
				a+=1;
			}
			else {
				a-=1;
			}
		}
		prec6=data[0];
		close(fd);
		close (fd2);
	}
	return 0;
}

void * affichage (void * b){
	while(1){
		usleep(500000);
		printf("%i\n",a);
	}
	return NULL;
}

int main (int argc, char * argv[]){
	pthread_t t1,t2;
	pthread_create(&t1,NULL,moteur,NULL);
	pthread_create(&t2,NULL,affichage,NULL);
	pthread_join(t1,NULL);
	pthread_join(t2,NULL);
	return 0;
}

