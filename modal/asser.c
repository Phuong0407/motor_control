#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <signal.h>

int a = 0;
#define SA 26
#define SB 27
int voulu=30;
float k=80;
float commande = 0;


char d_on[]="1";
char d_off[]="0";
int run =1;

void stop(int b)
{
	int fd= open("/sys/class/gpio/gpio13/value", O_WRONLY);
	write(fd,d_off,1);
	close (fd);
	run=0;
	printf("quitter!\n"); 
}

void myInterrupt(void){
		int res=digitalRead(SB);
		if (res==0){
			a-=1;
		}
		else {
			a+=1;
	    }
}

void * mot (void *b){

	int fd;
	int fdir;
	while (run){
		int t = abs((int)commande);
		if (t>5000){
			t=5000;
		}
		fdir= open("/sys/class/gpio/gpio12/value", O_WRONLY);
		if (commande <0){
			write (fdir,d_off,1);
		}
		else {
			write (fdir,d_on,1);
		}
		close (fdir);
		fd= open("/sys/class/gpio/gpio13/value", O_WRONLY);
			write(fd,d_on,1);
			usleep(t);
			write(fd,d_off,1);
			usleep(5000-t);
		close (fd);
		}
}

void * assert (void * b){
	while(run){
		int erreur = voulu-a;
		commande=(float)k*erreur;
		usleep(10000);
  }
}


int main (int argc, char * argv[]){
	wiringPiSetupGpio();
	signal(SIGINT,stop);
	pinMode (SA, INPUT);
	pinMode (SB, INPUT);
    wiringPiISR (SA, INT_EDGE_RISING, &myInterrupt);
    pthread_t t1,t2;
	pthread_create(&t1,NULL,mot,NULL);
	pthread_create(&t2,NULL,assert,NULL);
	while(run)
	{
		printf("erreur : %d\n", voulu-a);
		usleep(100000);
	}
	pthread_join(t1,NULL);
	pthread_join(t2,NULL);
	return 0;
}
