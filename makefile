CXX = g++ $(CXXFLAGS)
CXXFLAGS += -std=c++11 -isystem /usr/include/eigen3
LIBS = -lopencv_core \
			 -lopencv_features2d \
			 -lopencv_highgui \
			 -lopencv_imgproc \
			 -lopencv_legacy \
			 -lopencv_nonfree

OBJECT = orb sift surf features.o

all: $(OBJECT)

features.o: Features.h Features.cpp
	$(CXX) Features.cpp -c -o features.o

orb: ORB.cpp
	$(CXX) ORB.cpp -o orb $(LIBS)

sift: SIFT.cpp
	$(CXX) SIFT.cpp -o sift $(LIBS)

surf: SURF.cpp
	$(CXX) SURF.cpp -o surf $(LIBS)

test: UnitTests.cpp features.o
	$(CXX) UnitTests.cpp features.o -o test ${LIBS}

clean:
	rm -f $(OBJECT) test
