#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>
#include <pthread.h>
#include <wiringPi.h>

#define	LED1	16
#define	LED2	18
#define	SA	26
#define	SB	27
#define	DIR	12
#define	EN	13


char d_on[]="1";
char d_off[]="0";
int timer;

void * mot (void* a){
	char d_on[]="1";
	char d_off[]="0";
	int t=600;
	int fd;
	int fdir;
	int n;
	while (1){
		n=0;
		//fd= open("/sys/class/gpio/gpio13/value", O_WRONLY);
		//fdir= open("/sys/class/gpio/gpio12/value", O_WRONLY);
		while (n<500){
			digitalWrite (DIR,HIGH);
			digitalWrite(EN,HIGH);
			usleep(t);
			digitalWrite(EN,LOW);
			usleep(5000-t);
			n+=1;
		}
		while (n<1000){
			digitalWrite (DIR,LOW);
			digitalWrite(EN,HIGH);
			usleep(t);
			digitalWrite(EN,LOW);
			usleep(5000-t);
			n+=1;
		}
		//close(fd);
		//close(fdir);
	}
	return 0;
}

void * led1 (void* a){
	char data[36];
	char data2[36];
	int fd,fd2,fled;
	int prec6=1;
	int sens = 1;
	while (1){
		//fd= open("/sys/class/gpio/gpio26/value", O_RDONLY);
		//fd2 = open("/sys/class/gpio/gpio27/value", O_RDONLY);
		int res = digitalRead(SA);
		if (res==1 && prec6==0){
			int res2 = digitalRead(SB);
			if (res2==1 && sens==1){
				sens=0;
				//fled= open("/sys/class/gpio/gpio16/value", O_WRONLY);
				digitalWrite(LED1, HIGH);
				//close(fled);
			}
			else if (res2==0 && sens==0){
				sens=1;
				//fled= open("/sys/class/gpio/gpio16/value", O_WRONLY);
				digitalWrite(LED1,LOW);
				//close(fled);
			}
		}
		prec6=res;
		//close(fd);
		//close (fd2);
	}
	return 0;
}


void * led2 (void* a){
	char data[36];
	char data2[36];
	int prec6=1;
	int sens = 1;
	while (1){
		digitalWrite(LED2, HIGH);
		usleep(T);
		digitalWrite(LED2, LOW);
		usleep(10000-T);
		}
	return 0;
}

void * temps (void* a){
	char data[36];
	char data2[36];
	int fd,fd2,fled;
	int prec6=1;
	int sens = 1;
	while (timer=1){
		int res = digitalRead(SA);
		if (res==1 && prec6==0){
			int res2 = digitalRead(SB);
			if (res2==1 && sens==1){
				sens=0;
				//fled= open("/sys/class/gpio/gpio16/value", O_WRONLY);
				digitalWrite(LED1, HIGH);
				//close(fled);
			}
			else if (res2==0 && sens==0){
				sens=1;
				//fled= open("/sys/class/gpio/gpio16/value", O_WRONLY);
				digitalWrite(LED1,LOW);
				//close(fled);
			}
		}
		prec6=res;
		//close(fd);
		//close (fd2);
	}
	return 0;
}


int main (int argc, char * argv[]){
	wiringPiSetupGpio();
	pinMode (LED1, OUTPUT);
	pinMode (LED2, OUTPUT);
	pinMode (SA, INPUT);
	pinMode (SB, INPUT);
	pinMode (DIR, OUTPUT);
	pinMode (EN, OUTPUT);
	pthread_t t1,t2;
	pthread_create(&t1,NULL,mot,NULL);
	pthread_create(&t2,NULL,led1,NULL);
	pthread_join(t1,NULL);
	pthread_join(t2,NULL);
	return 0;
}
