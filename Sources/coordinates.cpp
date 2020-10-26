#include "coordinates.h"

Coordinates::Coordinates()
{

}

bool Coordinates::calibrateRobotCoordinates(std::vector<std::pair<int, int> > cameraXandYCoordinates, std::vector<std::pair<int, int> > robotXandYCoordinates)
{
    if (cameraXandYCoordinates.size() != robotXandYCoordinates.size() || cameraXandYCoordinates.empty()) {
        return false;
    }

    calibrationCount = cameraXandYCoordinates.size();

    for (int i = 0; i < calibrationCount; i++) {
        calibrationParameterX += robotXandYCoordinates[i].first - cameraXandYCoordinates[i].first;
        calibrationParameterY += robotXandYCoordinates[i].second - cameraXandYCoordinates[i].second;
    }

    calibrationParameterX = calibrationParameterX / calibrationCount;
    calibrationParameterY = calibrationParameterY / calibrationCount;

}


