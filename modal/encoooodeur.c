#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>

int main (int argc, char * argv[]){
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
			if (data2[0]=='1' && sens==1){
				sens=0;
				printf("horaire\n");
			}
			else if (data2[0]=='0' && sens==0){
				sens=1;
				printf("antihoraire\n");
			}
		}
		prec6=data[0];
		close(fd);
		close (fd2);
	}
	return 0;
}

