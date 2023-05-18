.PHONY: all build clean run debug rebuild demo

CFLAGS = -std=c++17 -Wall -Werror -Wfatal-errors -g3 -I"./Eigen/"

SRC = kalman.cpp

all: build

build: main.exe

main.exe: assignment3.cpp $(SRC)
	g++ $(CFLAGS) $^ -o $@

clean:
	rm -rf *.o *.exe

rebuild: clean build

run: main.exe
	./$<

debug: main.exe
	gdb ./$<

demo: demo_kalman_app.exe
	./$<

demo_kalman_app.exe: demo_kalman_app.cpp $(SRC)
	g++ $(CFLAGS) $^ -o $@

