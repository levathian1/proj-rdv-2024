CC = g++

proj: 
	mkdir -p build
	g++ main.cpp -o build/main -lm