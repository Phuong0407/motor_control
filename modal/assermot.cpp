#include "def.h"
extern int x;
extern int y;
extern int pixel;
int etape=1;
int low_H;
int low_S;
int low_V;
int high_H;
int high_S;
int high_V;
int erosion_size;
int dilation_size;


/////////////////////////////////////////////////////////
// varibales changeables : 
//int voulu1=0;
int sens1=1;
int voulu2=-280;
int sens2=0;
int voulu3=280;
int sens3=1;
int voulurot1=200;
int voulurot2=0;
int voulurot3=370;
int srot=1;
int modrot1;
int modrot2;
int modrot3;

float ki1=0.6;
float ki2=0.6;
float ki3=0.6;
float kd1=0.1;
float kd2=0.1;
float kd3=0.1;
float kp1=4;
float kp2=4;
float kp3=4;
float k_dev=0.0000001;
int k_devlin=3;
int k_der=7;
////////////////////////////////////////////////////////
//paramètres communs : 

char d_on[]="1";
char d_off[]="0";
float ki=0.8;
float kd=0;
int run = 1;
int seuil = 1800;


int modele (int v){
	int vitt=abs(v);
	if (vitt<124){
		return 0;
	}
	else {
		if (v>0){
			return (int)(vitt-124)/0.247;
		}
		if (v<0){
			return (int)(124-vitt)/0.247;
		}
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
int powo;
int erreur1;


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
	//etape 1 : 
		if (etape==1 || etape==3 || etape==5||etape==6|| etape==8){
			t1 = (int)commande1 + modrot1*90/100;
			fdir= open("/sys/class/gpio/gpio12/value", O_WRONLY);
			if (t1<0){
				sens1=0;
				write (fdir,d_off,1);
			}
			else {
				sens1=1;
				write (fdir,d_on,1);
			}
			t1=abs(t1);
			if (t1>5000){
				t1=5000;
			}
			close (fdir);
			fd = open("/sys/class/gpio/gpio13/value", O_WRONLY);
			write(fd,d_on,1);
			usleep(t1);
			write(fd,d_off,1);
			usleep(5000-t1);
		}
	//etape 2 : 
		if (etape==2 || etape==4 || etape==7|| etape==9){
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
		}
		close (fd);
		}
}

void * assert1 (void * b){
	while(run){
	//etape 1 : 
	if (etape==1 || etape==3 || etape==5||etape==6|| etape==8){
		old1=a1;
		usleep(100000);
		new1=a1;
		vitold1 = vit1;
		vit1 = (new1-old1)/0.1;
		erreur1=voulurot1-vit1;
		etot1=etot1+(erreur1)*0.1;
		der1 = (vit1-vitold1)/0.1;
		commande1=(float)ki1*etot1 + (float)kd1*der1 +(float)kp1*erreur1;
	}
	//etape 2 : 
	if (etape==2 || etape==4 || etape==7|| etape==9){
		erreur_bary=-(x-639);
		powo=pow(erreur_bary,5);
		commande1=abs(k_der*erreur_bary);
		if (erreur_bary>0){
			sens1=0;
		}
		else {
			sens1=1;
		}
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
		if (etape==1 || etape==3 || etape==5||etape==6|| etape==8){
			t2 = (int)commande2 + modrot2*90/100 ;
		}
		if (etape==2 || etape==4 || etape==7|| etape==9){
			t2 = (int)commande2 + mod2*90/100 - k_dev*powo - k_devlin*erreur_bary;
		}
		fdir= open("/sys/class/gpio/gpio22/value", O_WRONLY);
		if (t2<0){
			sens2=0;
			write (fdir,d_off,1);
		}
		else {
			sens2=1;
			write (fdir,d_on,1);
		}
		t2=abs(t2);
		if (t2>5000){
			t2=5000;
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
		vit2 = (new2-old2)/0.1;
		if (etape==1 || etape==3 || etape==5||etape==6|| etape==8){
			erreur2=voulurot2-vit2;
		}
		if (etape==2 || etape==4 || etape==7|| etape==9){
			erreur2=voulu2-vit2;
		}
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
		if (etape==1 || etape==3 || etape==5||etape==6|| etape==8){
			t3 = (int)commande3 + modrot3*90/100;
		}
		if (etape==2 || etape==4 || etape==7|| etape==9){
			t3 = (int)commande3 + mod3*90/100 - k_dev*powo - k_devlin*erreur_bary;
		}
		fdir= open("/sys/class/gpio/gpio5/value", O_WRONLY);
		if (t3<0){
			sens3=0;
			write (fdir,d_off,1);
		}
		else {
			sens3=1;
			write (fdir,d_on,1);
		}
		t3=abs(t3);
		if (t3>5000){
			t3=5000;
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
		vit3 = (new3-old3)/0.1;
		if (etape==1 || etape==3 || etape==5||etape==6|| etape==8){
			erreur3=voulurot3-vit3;
		}
		if (etape==2 || etape==4 || etape==7|| etape==9){
			erreur3=voulu3-vit3;
		}
		etot3=etot3+(erreur3)*0.1;
		der3 = (vit3-vitold3)/0.1;
		commande3=(float)ki3*etot3 + (float)kd3*der3 +(float)kp3*erreur3;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char * argv[]){
	low_H=48;
	low_S=63;
	low_V=75;
	high_H=76;
	high_S=247;
	high_V=206;
	erosion_size=4;
	dilation_size=4;
	seuil=1800;
	etot1=0;
	etot2=0;
	etot3=0;
	modrot1=modele(voulurot1);
	modrot2=modele(voulurot2);
	modrot3=modele(voulurot3);
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
	pthread_create(&cam,NULL,recup_bary_seuil,NULL);
	usleep(5000000);
	pthread_create(&c1,NULL,mot1,NULL);
	pthread_create(&c2,NULL,assert1,NULL);
	pthread_create(&c3,NULL,mot2,NULL);
	pthread_create(&c4,NULL,assert2,NULL);
	pthread_create(&c5,NULL,mot3,NULL);
	pthread_create(&c6,NULL,assert3,NULL);
	
	while(run)
	{
		//chercher 1er objet
		if (etape==1 && pixel>seuil){
			printf("fin 1");
			etape=2;
		}
		//avancer vers
		if (etape==2 && pixel<seuil){
			printf("fin 2");
		etape=3;
			low_H=3;
			low_S=88;
			low_V=135;
			high_H=8;
			high_S=240;
			high_V=242;
			erosion_size=7;
			dilation_size=9;
			seuil=20000;
		}
		//chercher 1er zone
		if (etape==3 && pixel>seuil){
			printf("fin 3");
			etape=4;
		}
		//avancer vers
		if (etape==4 && pixel>800000){
			//reculer
			printf("fin 4");
			etape=5;
			voulurot1=0;
			voulurot2=550;
			voulurot3=-550;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(5000000);
			//chercher 2e objet
			voulurot1=200;
			voulurot2=0;
			voulurot3=370;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			low_H=119;
			low_S=156;
			low_V=23;
			high_H=127;
			high_S=211;
			high_V=179;
			erosion_size=4;
			dilation_size=18;
			printf("fin 5");
			etape=6;
			seuil=2900;
			usleep(2000000);
		}
		//avancer vers
		if (etape==6 && pixel>seuil){
			printf("fin 6");
			etape=7;
		}
		//chercher 2e zone
		if (etape==7 && pixel<1000){
			printf("fin 7");
			low_H=168;
			low_S=110;
			low_V=164;
			high_H=180;
			high_S=232;
			high_V=255;
			erosion_size=6;
			dilation_size=16;
			etape=8;
			seuil=40000;
		}
		//avancer vers
		if (etape==8 && pixel>seuil){
			printf("fin 8");
			etape=9;
		}
		//stopper tout
		if (etape==9 && pixel>810000){
			printf("fin 9");
			//reculer
			printf("fin 4");
			etape=5;
			voulurot1=0;
			voulurot2=550;
			voulurot3=-550;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(5000000);
			//danser
			voulurot1=-300;
			voulurot2=-1250;
			voulurot3=0;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=300;
			voulurot2=1250;
			voulurot3=0;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=300;
			voulurot2=0;
			voulurot3=1250;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=-300;
			voulurot2=-0;
			voulurot3=-1250;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			
			voulurot1=-300;
			voulurot2=-1250;
			voulurot3=0;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=300;
			voulurot2=1250;
			voulurot3=0;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=300;
			voulurot2=0;
			voulurot3=1250;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=-300;
			voulurot2=-0;
			voulurot3=-1250;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			
			voulurot1=-300;
			voulurot2=-1250;
			voulurot3=0;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=300;
			voulurot2=1250;
			voulurot3=0;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=300;
			voulurot2=0;
			voulurot3=1250;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			voulurot1=-300;
			voulurot2=-0;
			voulurot3=-1250;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(500000);
			
			voulurot1=-900;
			voulurot2=-900;
			voulurot3=-900;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(1500000);
			
			voulurot1=1200;
			voulurot2=1200;
			voulurot3=1200;
			modrot1=modele(voulurot1);
			modrot2=modele(voulurot2);
			modrot3=modele(voulurot3);
			usleep(2500000);
			run=0;
		}
		/*printf("vitesse1 : (nicht valeur) %d\n", vit1);
		printf("vitesse2 : %d\n", vit2);
		printf("vitesse3 :  %d\n", vit3);
		//printf("etot : %f\n", etot2);
		//printf("erreur : %f\d", etot);
		printf("t1 : %d\n", t1);
		printf("t2 : %d\n", t2);
		printf("t3 : %d\n", t3);
		printf("powo : %d\n", powo);
		printf("\n");*/
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





