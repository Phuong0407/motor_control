all : asser asservit camera exe exa dilatation dib

asser : asser.c
	gcc asser.c -o asser -lpthread -lwiringPi
	
asservit : asservit.c
	gcc asservit.c -o asservit -lpthread -lwiringPi
	
camera : camera.cpp
	g++ camera.cpp -o camera `pkg-config --libs --cflags opencv4`

dilatation : dilatation.cpp
	g++ dilatation.cpp -o dilatation `pkg-config --libs --cflags opencv4`
	
exe : asservitrob.o camerarob.o
	g++ -o $@ $^ -lpthread -lwiringPi `pkg-config --libs opencv4`
	
camerarob.o : camerarob.cpp
	g++ -c camerarob.cpp `pkg-config --cflags opencv4`
	
asservitrob.o : asservitrob.cpp
	g++ -c asservitrob.cpp `pkg-config --cflags opencv4`




exa : assermot.o rechbary.o
	g++ -o $@ $^ -lpthread -lwiringPi `pkg-config --libs opencv4`
	
rechbary.o : rechbary.cpp
	g++ -c rechbary.cpp `pkg-config --cflags opencv4`
	
assermot.o : assermot.cpp
	g++ -c assermot.cpp `pkg-config --cflags opencv4`



dib : dilavecbarres.cpp
	g++ dilavecbarres.cpp -o dilavecbarres `pkg-config --libs --cflags opencv4`
