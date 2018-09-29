all: random

random: main.o Planner.o header.h
	g++ -std=c++11 header.h Planner.o main.o -o random
	
main.o: main.cpp
	g++ -std=c++11 -c main.cpp

Planner.o: Planner.cpp
	g++ -std=c++11 -c Planner.cpp
	
clean:
	rm -rf *o random
