#ifndef COORDINATES_H
#define COORDINATES_H
#include <vector>
#include <utility>


class Coordinates
{
public:
    Coordinates();

    bool calibrateRobotCoordinates(std::vector<std::pair<int, int>> cameraXandYCoordinates, std::vector<std::pair<int, int>> robotXandYCoordinates);

private:
    int robotX, robotY, cameraX, cameraY, calibrationCount;
    double calibrationParameterX{0}, calibrationParameterY{0};
};

#endif // COORDINATES_H
