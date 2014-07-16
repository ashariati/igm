VRPNPATH = ../vrpn
LIBOVRPATH = ../LibOVR
GLFWPATH = ../glfw-3.0.4
GLEWPATH = ../glew-1.10.0
GLMPATH = ../glm
ASSIMPPATH = ../assimp-3.1.1/

INCPATH = -I./include \
		  -I/usr/include \
		  -I$(GLMPATH) \
		  -I$(GLEWPATH)/include \
		  -I$(GLFWPATH)/include \
		  -I$(LIBOVRPATH)/Include \
		  -I$(LIBOVRPATH)/Src \
		  -I$(VRPNPATH) \
		  -I$(ASSIMPPATH)/include/

OBJPATH = ./obj
BINPATH = ./bin
SRCPATH = ./src

LIBS = -L/usr/lib \
	   -L$(VRPNPATH)/pc_linux64 \
	   -L$(LIBOVRPATH)/Lib/Linux/Release/x86_64 \
	   -L$(GLFWPATH)/build/src \
	   -L$(GLEWPATH)/lib \
	   -L$(ASSIMPPATH)/build/code \
	   -Wl -Bstatic \
	   -lovr \
	   -lvrpn \
	   -lglfw3 \
	   -Wl -Bdynamic \
	   -lGL \
	   -lGLU \
	   -lglut \
	   -lGLEW \
	   -lXi \
	   -lXrandr \
	   -lXxf86vm \
	   -lX11 \
	   -lm \
	   -lpthread \
	   -ludev \
	   -lboost_thread-mt \
	   -lassimp

OBJECTS = $(OBJPATH)/MarkOneOGL.o \
		  $(OBJPATH)/objloader.o \
		  $(OBJPATH)/texture.o \
		  $(OBJPATH)/shader.o

TARGET = $(BINPATH)/MarkOneOGL

$(TARGET): $(OBJECTS)
	g++ -Wall -o $(TARGET) $(OBJECTS) $(LIBS)

$(OBJPATH)/MarkOneOGL.o: $(SRCPATH)/MarkOneOGL.cpp
	g++ -Wall -c $(INCPATH) -o $(OBJPATH)/MarkOneOGL.o $(SRCPATH)/MarkOneOGL.cpp

$(OBJPATH)/objloader.o: $(SRCPATH)/objloader.cpp
	g++ -Wall -c $(INCPATH) -o $(OBJPATH)/objloader.o $(SRCPATH)/objloader.cpp

$(OBJPATH)/texture.o: $(SRCPATH)/texture.cpp
	g++ -Wall -c $(INCPATH) -o $(OBJPATH)/texture.o $(SRCPATH)/texture.cpp

$(OBJPATH)/shader.o: $(SRCPATH)/shader.cpp
	g++ -Wall -c $(INCPATH) -o $(OBJPATH)/shader.o $(SRCPATH)/shader.cpp

clean:
	rm -f $(OBJPATH)/*
	rm -f $(BINPATH)/*
