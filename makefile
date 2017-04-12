CXX = g++-5 $(CXXFLAGS)
CXXFLAGS += -std=c++11 \
						-I. \
						-I../msckf \
						-isystem /usr/include/eigen3 \
						-Ofast
LIBS = -L/usr/local/lib -Wl,-rpath,/usr/local/lib,--as-needed \
			 -lopencv_core \
			 -lopencv_features2d \
			 -lopencv_highgui \
			 -lopencv_imgproc \
			 -lopencv_legacy \
			 -lopencv_nonfree \
			 -lboost_system \
			 -lboost_filesystem \
			 -lGL -lGLU -lGLEW -lglut

OBJECTS = features.o \
					orb \
					sift \
					surf \
					msckf.o \
					feature_tracker.o \
					slam_drawer.o \
					kitti_track \
					unit_test \
					unit_test2 \
					unit_test3

all: $(OBJECTS)

process_images: FeatureMatcher.h features.o ProcessImages.cpp
	$(CXX) ProcessImages.cpp features.o -o process_images $(LIBS)

features.o: Features.h Features.cpp
	$(CXX) Features.cpp -c -o features.o $(LIBS)

orb: ORB.cpp
	$(CXX) ORB.cpp -o orb $(LIBS)

sift: SIFT.cpp
	$(CXX) SIFT.cpp -o sift $(LIBS)

surf: SURF.cpp
	$(CXX) SURF.cpp -o surf $(LIBS)

msckf.o: ../msckf/MSCKF/MSCKF.h ../msckf/MSCKF/MSCKF.cpp
	$(CXX) ../msckf/MSCKF/MSCKF.cpp -c -o msckf.o

feature_tracker.o: FeatureMatcher.h Features.h Utils.h KITTIFeatureTracker.h KITTIFeatureTracker.cpp
	$(CXX) KITTIFeatureTracker.cpp -c -o feature_tracker.o $(LIBS)

slam_drawer.o: SLAMTrajectoryDrawer.h SLAMTrajectoryDrawer.cpp
	$(CXX) SLAMTrajectoryDrawer.cpp -c -o slam_drawer.o $(LIBS)

kitti_track: TrackKITTIFeatures.cpp features.o feature_tracker.o
	$(CXX) TrackKITTIFeatures.cpp features.o feature_tracker.o -o kitti_track $(LIBS)

unit_test: unit_tests/UnitTest.cpp features.o feature_tracker.o slam_drawer.o msckf.o
	$(CXX) unit_tests/UnitTest.cpp features.o feature_tracker.o slam_drawer.o msckf.o -o unit_test $(LIBS)

unit_test2: unit_tests/UnitTest2.cpp features.o msckf.o
	$(CXX) unit_tests/UnitTest2.cpp msckf.o features.o -o unit_test2 $(LIBS)

unit_test3: unit_tests/UnitTest3.cpp slam_drawer.o
	$(CXX) unit_tests/UnitTest3.cpp slam_drawer.o -o unit_test3 $(LIBS)

clean:
	rm -f $(OBJECTS) python/*.pyc
