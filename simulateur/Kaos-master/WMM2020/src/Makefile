CC = gcc
CFLAGS = -g -O2 -Wall -W
LDFLAGS =
LIBS = -lm

all: MagnetoCompiledLinux

MagnetoCompiledLinux: 
	${CC} ${LDFLAGS} CodeMagneto.c -o MagnetoCompiledLinux ${LIBS}

MagnetoCompiledWindows:
	x86_64-w64-mingw32-gcc CodeMagneto.c -o MagnetoCompiledWindows -lm

clean:
	rm -f *.o
	rm -f MagnetoCompiledLinux
