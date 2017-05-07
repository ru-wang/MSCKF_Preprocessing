#include "SLAMTrajectoryDrawer.h"

/* unit test */
int main(int argc, char* argv[]) {
  for (int i = 1; i < argc; ++i) {
    char* tractory_file = argv[i];
    SLAMTrajectoryDrawer::ReadTrajectoryFromFile(tractory_file);
  }

  SLAMTrajectoryDrawer::SetupGLUT(argc, argv);

  SLAMTrajectoryDrawer::SetupGLEW();

  SLAMTrajectoryDrawer::SetupGLSL();

  SLAMTrajectoryDrawer::StartDrawing();

  SLAMTrajectoryDrawer::FreeTrajectory();

  return 0;
}
