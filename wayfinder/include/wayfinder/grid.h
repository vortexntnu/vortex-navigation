#include <vector>
#include <cmath>
#include <memory>

struct Point{
    int x;
    int y;
    int z;
};

struct GridPixel{
    double value;
    double depth; //dummy variable for 2.5D grid
};

struct Params{
    double sigmaX;
    double sigmaY;
    double sigmaZ;
    int offsetX;
    int offsetY;
    int offsetZ;
};

class Grid{
    //ever expanding grid with obstacles. centered at center
    public:
        Params params = {20, 20, 20, 0, 0, 0};

        Point center;
        std::unique_ptr<std::vector<std::vector<std::vector<GridPixel>>>> grid; //negative values are obstacles

        Grid(int size = 200);
        void setParams(Params newParams);
        void updateGrid(std::vector<Point> wallPoints); //updates the grid with new readings from the sensors
        Point getWaypoint(); //retives the next waypoint. 

    private:
        void initGaussian(std::unique_ptr<std::vector<std::vector<std::vector<GridPixel>>>>& gaussGrid); //initializes the gaussian distribution
        void resizeGrid(); //doubles the size of the grid if values fall outside the grid size
};
