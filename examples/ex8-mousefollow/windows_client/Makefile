# This Makefile makes the window binary

mouse_follow.exe: windows-main.c windows-resources.o
	i586-mingw32msvc-gcc -mwindows windows-main.c windows-resources.o -lws2_32 -o mouse_follow.exe

windows-resources.o: windows-resources.rc
	i586-mingw32msvc-windres windows-resources.rc -o windows-resources.o

clean:
	rm -f *.o mouse_follow.exe
