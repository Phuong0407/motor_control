#include <gpiod.h>
#include <unistd.h>

#include <stdio.h>
const char *chipname = "gpiochip0";
 
struct gpiod_chip *chip;
struct gpiod_line *lineRed;
struct gpiod_line *lineButton;

int main() {
    chip = gpiod_chip_open_by_name(chipname);
    lineRed = gpiod_chip_get_line(chip, 13);
    lineButton = gpiod_chip_get_line(chip, 12);
    gpiod_line_request_output(lineRed, "example1", 0);
    gpiod_line_request_input(lineButton, "example1");
    int i = 1;
    do {
	gpiod_line_set_value(lineRed, 1);
	usleep(1500);
	gpiod_line_set_value(lineRed, 0);
	usleep(8500);
		//if (val == 0) {
			//printf("val : %d \n",  val);
			//break;
			//;
		//}
			//usleep(10000);
		
		
		
	i++;
	} while (i < 10000);
}
