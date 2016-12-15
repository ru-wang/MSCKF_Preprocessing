CXX = g++-5 $(CXXFLAGS)
CXXFLAGS += -std=c++11
LIBS = -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
			 -lopencv_core \
			 -lopencv_features2d \
			 -lopencv_highgui \
			 -lopencv_imgproc \
			 -lopencv_legacy \
			 -lopencv_nonfree \
			 -lboost_system \
			 -lboost_filesystem

OBJECT = process_images orb sift surf features.o

all: $(OBJECT)

process_images: FeatureMatcher.h features.o ProcessImages.cpp
	$(CXX) ProcessImages.cpp features.o -o process_images $(LIBS)

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
