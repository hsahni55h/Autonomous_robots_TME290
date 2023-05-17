.PHONY: all build clean run debug rebuild

CFLAGS = -std=c++17 -Wall -Werror $^ -g3 -I"./Eigen/"

all: build

build: main.exe

main.exe: multidimensional_kalman_filter.cpp
	g++ $(CFLAGS) -o $@

clean:
	rm -rf *.o *.exe

rebuild: clean build

run: main.exe
	./$<

debug: main.exe
	gdb ./$<
