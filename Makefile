CXXFLAGS =	-O3 -std=c++11 -g -Wall -pthread -fmessage-length=0 -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/qt4/QtOpenGL

OBJS =		homework_sim.o graphics.o moc_gr.o default_actor.o

LIBS =		lib/libsimulator.a -lGL -lGLU -lglut -lQtGui -lQtCore -lQtOpenGL

TARGET =	homework_sim


$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

default: $(TARGET)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

moc_gr.cpp: graphics.h
	moc $(DEFINES) $(INCPATH) $< -o $@
