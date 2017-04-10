#include "SLAMTrajectoryDrawer.h"

/* Unit test */
int main(int argc, char* argv[]) {
  char* tractory_file = argv[1];
  char* tractory_file2 = nullptr;
  if (argc > 2)
    tractory_file2 = argv[2];

  SLAMTrajectoryDrawer::ReadTrajectoryFromFile(tractory_file, tractory_file2);

  SLAMTrajectoryDrawer::SetupGLUT(argc, argv);

  SLAMTrajectoryDrawer::SetupGLEW();

  SLAMTrajectoryDrawer::SetupGLSL();

  SLAMTrajectoryDrawer::StartDrawing();

  SLAMTrajectoryDrawer::FreeTrajectory();

  return 0;
}
