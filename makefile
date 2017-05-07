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
					generic_tracker.o \
					utils.o \
					kitti_track \
					unit_test \
					unit_test2 \
					unit_test3 \
					unit_test4

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

msckf.o: ../msckf/MSCKF/MSCKF.h ../msckf/MSCKF/MSCKF.cpp
	$(CXX) ../msckf/MSCKF/MSCKF.cpp -c -o msckf.o

generic_tracker.o: FeatureMatcher.h Features.h GenericFeatureTracker.h GenericFeatureTracker.cpp
	$(CXX) GenericFeatureTracker.cpp -c -o generic_tracker.o

feature_tracker.o: Exception.h Features.h GenericFeatureTracker.h Utils.h KITTIFeatureTracker.h KITTIFeatureTracker.cpp
	$(CXX) KITTIFeatureTracker.cpp -c -o feature_tracker.o

slam_drawer.o: SLAMTrajectoryDrawer.h SLAMTrajectoryDrawer.cpp Utils.h
	$(CXX) SLAMTrajectoryDrawer.cpp -c -o slam_drawer.o

utils.o: Utils.h Utils.cpp
	$(CXX) Utils.cpp -c -o utils.o

kitti_track: TrackKITTIFeatures.cpp features.o feature_tracker.o generic_tracker.o utils.o
	$(CXX) TrackKITTIFeatures.cpp features.o feature_tracker.o generic_tracker.o utils.o -o kitti_track $(LIBS)

unit_test: unit_tests/UnitTest.cpp ../msckf/MSCKF/JPL.h ../msckf/MSCKF/MSCKF.h features.o feature_tracker.o generic_tracker.o slam_drawer.o msckf.o
	$(CXX) unit_tests/UnitTest.cpp features.o feature_tracker.o generic_tracker.o slam_drawer.o msckf.o -o unit_test $(LIBS)

unit_test2: unit_tests/UnitTest2.cpp ../msckf/MSCKF/JPL.h ../msckf/MSCKF/MSCKF.h msckf.o
	$(CXX) unit_tests/UnitTest2.cpp msckf.o -o unit_test2 $(LIBS)

unit_test3: unit_tests/UnitTest3.cpp slam_drawer.o utils.o
	$(CXX) unit_tests/UnitTest3.cpp slam_drawer.o utils.o -o unit_test3 $(LIBS)

unit_test4: unit_tests/UnitTest4.cpp ../msckf/MSCKF_Simulation/Helpers.h slam_drawer.o msckf.o
	$(CXX) unit_tests/UnitTest4.cpp slam_drawer.o msckf.o -o unit_test4 $(LIBS)

clean:
	rm -f $(OBJECTS) python/*.pyc
