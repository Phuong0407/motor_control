#include "define.h"
extern int x;
/////////////////////////////////////////////////////////
// varibales changeables : 
//int voulu1=0;
int sens1=1;
int voulu2=300;
int sens2=0;
int voulu3=300;
int sens3=1;

//float ki1=0.6;
float ki2=0.6;
float ki3=0.6;
//float kd1=0.1;
float kd2=0.1;
float kd3=0.1;
//float kp1=4;
float kp2=4;
float kp3=4;
int k_bary=6;
////////////////////////////////////////////////////////
//paramètres communs : 

char d_on[]="1";
char d_off[]="0";
float ki=0.8;
float kd=0;
int run = 1;


int modele (int v){
	if (v<124){
		return 0;
	}
	else {
		return (int)(v-124)/0.247;
	}
}

void stop(int b)
{
	int fd= open("/sys/class/gpio/gpio13/value", O_WRONLY);
	write(fd,d_off,1);
	close (fd);
	fd= open("/sys/class/gpio/gpio23/value", O_WRONLY);
	write(fd,d_off,1);
	close (fd);
	fd= open("/sys/class/gpio/gpio6/value", O_WRONLY);
	write(fd,d_off,1);
	close (fd);
	run=0;
	printf("quitter!\n"); 
}
//////////////////////////////////////////////////////////////////

//variables moteur 1 : 

int a1 = 0;
int t1=1000;
int old1=0;
int new1=0;
#define SA1 26
#define SB1 27
float commande1 = 0;
int vit1=0;
float etot1 = 0;
int mod1= 0;
int vitold1;
int der1;
int erreur_bary;


//programmes moteur 1: 

void myInterrupt1(void){
		int res1=digitalRead(SB1);
		if (res1==0){
			a1-=1;
		}
		else {
			a1+=1;
	    }
}

void * mot1 (void *b){
	int fd;
	int fdir;
	while (run){
		t1 = commande1;
		if (t1>5000){
			t1=5000;
		}
		if (t1<0){
			t1=0;
		}
		fdir= open("/sys/class/gpio/gpio12/value", O_WRONLY);
		if (sens1 == 0){
			write (fdir,d_off,1);
		}
		else {
			write (fdir,d_on,1);
		}
		close (fdir);
		fd = open("/sys/class/gpio/gpio13/value", O_WRONLY);
			write(fd,d_on,1);
			usleep(t1);
			write(fd,d_off,1);
			usleep(5000-t1);
		close (fd);
		}
}

void * assert1 (void * b){
	while(run){
		erreur_bary=x-639;
		commande1=abs(k_bary*erreur_bary);
		if (erreur_bary>0){
			sens1=0;
		}
		else {
			sens1=1;
		}
  }
}

/////////////////////////////////////////////////////////////////////////////////////////


// variables moteur 2 : 

int a2 = 0;
int t2=1000;
int old2=0;
int new2=0;
#define SA2 18
#define SB2 19
float commande2 = 0;
int vit2=0;
float etot2 = 0;
int mod2= 0;
int vitold2;
int der2;
int erreur2;

// programmes moteur 2 : 

void myInterrupt2(void){
		int res2=digitalRead(SB2);
		if (res2==0){
			a2-=1;
		}
		else {
			a2+=1;
	    }
}

void * mot2 (void *b){
	int fd;
	int fdir;
	while (run){
		t2 = (int)commande2 + mod2*80/100;
		if (t2>5000){
			t2=5000;
		}
		if (t2<0){
			t2=0;
		}
		fdir= open("/sys/class/gpio/gpio22/value", O_WRONLY);
		if (sens2 == 0){
			write (fdir,d_off,1);
		}
		else {
			write (fdir,d_on,1);
		}
		close (fdir);
		fd = open("/sys/class/gpio/gpio23/value", O_WRONLY);
			write(fd,d_on,1);
			usleep(t2);
			write(fd,d_off,1);
			usleep(5000-t2);
		close (fd);
		}
}

void * assert2 (void * b){
	while(run){
		old2=a2;
		usleep(100000);
		new2=a2;
		vitold2 = vit2;
		vit2 = abs((new2-old2)/0.1);
		erreur2=voulu2-vit2;
		etot2=etot2+(erreur2)*0.1;
		der2 = (vit2-vitold2)/0.1;
		commande2=(float)ki2*etot2 + (float)kd2*der2 +(float)kp2*erreur2;
  }
}

////////////////////////////////////////////////////////////////////////


// variables moteur 3 : 

int a3 = 0;
int t3=1000;
int old3=0;
int new3=0;
#define SA3 16
#define SB3 17
float commande3 = 0;
int vit3=0;
float etot3 = 0;
int mod3= 0;
int vitold3;
int der3;
int erreur3;



// programmes moteur 3 : 

void myInterrupt3(void){
		int res3=digitalRead(SB3);
		if (res3==0){
			a3-=1;
		}
		else {
			a3+=1;
	    }
}

void * mot3 (void *b){
	int fd;
	int fdir;
	while (run){
		t3 = (int)commande3 + mod3*80/100;
		if (t3>5000){
			t3=5000;
		}
		if (t3<0){
			t3=0;
		}
		fdir= open("/sys/class/gpio/gpio5/value", O_WRONLY);
		if (sens3 == 0){
			write (fdir,d_off,1);
		}
		else {
			write (fdir,d_on,1);
		}
		close (fdir);
		fd = open("/sys/class/gpio/gpio6/value", O_WRONLY);
			write(fd,d_on,1);
			usleep(t3);
			write(fd,d_off,1);
			usleep(5000-t3);
		close (fd);
		}
}

void * assert3 (void * b){
	while(run){
		old3=a3;
		usleep(100000);
		new3=a3;
		vitold3 = vit3;
		vit3 = abs((new3-old3)/0.1);
		erreur3=voulu3-vit3;
		etot3=etot3+(erreur3)*0.1;
		der3 = (vit3-vitold3)/0.1;
		commande3=(float)ki3*etot3 + (float)kd3*der3 +(float)kp3*erreur3;
  }
}


int main (int argc, char * argv[]){
	etot1=0;
	etot2=0;
	etot3=0;	
	//mod1 = modele(voulu1);
	mod2 = modele(voulu2);
	mod3 = modele(voulu3);
	wiringPiSetupGpio();
	signal(SIGINT,stop);
	pinMode (SA1, INPUT);
	pinMode (SB1, INPUT);
	pinMode (SA2, INPUT);
	pinMode (SB2, INPUT);
	pinMode (SA3, INPUT);
	pinMode (SB3, INPUT);
    wiringPiISR (SA1, INT_EDGE_RISING, &myInterrupt1);
    wiringPiISR (SA2, INT_EDGE_RISING, &myInterrupt2);
    wiringPiISR (SA3, INT_EDGE_RISING, &myInterrupt3);
    pthread_t c1,c2,c3,c4,c5,c6,cam;
	pthread_create(&c1,NULL,mot1,NULL);
	pthread_create(&c2,NULL,assert1,NULL);
	pthread_create(&c3,NULL,mot2,NULL);
	pthread_create(&c4,NULL,assert2,NULL);
	pthread_create(&c5,NULL,mot3,NULL);
	pthread_create(&c6,NULL,assert3,NULL);
	pthread_create(&cam,NULL,recuperation_barycentre,NULL);
	while(run)
	{
		printf("vitesse1 : %d\n", vit1);
		printf("vitesse2 : %d\n", vit2);
		printf("vitesse3 : (pas de valeur) %d\n", vit3);
		//printf("etot : %f\n", etot2);
		//printf("erreur : %f\d", etot);
		printf("t1 : %d\n", t1);
		printf("t2 : %d\n", t2);
		printf("t3 : %d\n", t3);
		printf("\n");
		usleep(500000);
	}
	pthread_join(c1,NULL);
	pthread_join(c2,NULL);
	pthread_join(c3,NULL);
	pthread_join(c4,NULL);
	pthread_join(c5,NULL);
	pthread_join(c6,NULL);
	pthread_join(cam,NULL);
	return 0;
}


