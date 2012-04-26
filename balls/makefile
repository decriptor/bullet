# Bullet balls
includes = $(wildcard *.h)
project5: main.cpp TheBalls.cpp MyOpenGL.cpp ${includes}
	g++ main.cpp TheBalls.cpp MyOpenGL.cpp -o project5 `pkg-config --cflags --libs bullet` -lglut -lGL -lm

clean:
	rm -f project5
