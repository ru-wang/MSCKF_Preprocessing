CXX = g++-5 $(CXXFLAGS)
CXXFLAGS += -std=c++11 -g \
						-I../msckf \
						-isystem /usr/include/eigen3
LIBS = -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
			 -lopencv_core \
			 -lopencv_features2d \
			 -lopencv_highgui \
			 -lopencv_imgproc \
			 -lopencv_legacy \
			 -lopencv_nonfree \
			 -lboost_system \
			 -lboost_filesystem \
			 -lGL -lGLU -lGLEW -lglut

OBJECTS = process_images orb sift surf features.o msckf.o test kitti_test draw_test

all: $(OBJECTS)

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

msckf.o: ../msckf/MSCKF/MSCKF.cpp
	$(CXX) ../msckf/MSCKF/MSCKF.cpp -c -o msckf.o

test: FeatureMatcher.h UnitTests.cpp features.o msckf.o
	$(CXX) UnitTests.cpp msckf.o features.o -o test ${LIBS}

kitti_test: FeatureMatcher.h TrackKITTIFeatures.cpp features.o
	$(CXX) TrackKITTIFeatures.cpp features.o -o kitti_test ${LIBS}

draw_test: SLAMTrajectoryDrawer.h SLAMTrajectoryDrawer.cc
	$(CXX) SLAMTrajectoryDrawer.cc -o draw_test ${LIBS}

clean:
	rm -f $(OBJECTS)
