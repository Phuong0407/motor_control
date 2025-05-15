#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

int b = 0;

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
			if (b==0){
				b=1;}
			else{
				b=0;}
		}
		prec=data[0];
		usleep(1000);
		}
	return NULL;
	}
	
void * servo (void * a){
	char d_on[]="1";
	char d_off[]="0";
	int t=600;
	int fden;
	int fdir;
	while (1){
		fden= open("/sys/class/gpio/gpio13/value", O_WRONLY);
		fdir= open("/sys/class/gpio/gpio12/value", O_WRONLY);
		if (b==1){
			write (fdir,d_on,1);
		}
		else{
			write (fdir,d_off,1);
		}
		write(fden,d_on,1);
		usleep(t);
		write(fden,d_off,1);
		close(fden);
		close(fdir);
		usleep(5000-t);
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

