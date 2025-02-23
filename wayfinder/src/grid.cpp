#include "grid.h"


Grid::Grid(int size){
    //initialize the grid to the size
    grid = std::make_unique<std::vector<std::vector<std::vector<double>>>>
        (size, std::vector<std::vector<double>>(size, std::vector<double>(size, 0)));
    center.x = size/2;
    center.y = size/2;
    center.z = size/2;
    initGaussian(grid);
}

void Grid::initGaussian(std::unique_ptr<std::vector<std::vector<std::vector<double>>>>& gaussGrid){
    for (int x = 0; x < grid->size(); x++){
        for (int y = 0; y < grid->size(); y++){
            for (int z = 0; z < grid->size(); z++){
                (*grid)[x][y][z] = exp(-1*(
                    pow(x-(center.x + params.offsetX), 2)/(2*pow(params.sigmaX, 2)) + 
                    pow(y-(center.y + params.offsetY), 2)/(2*pow(params.sigmaY, 2)) + 
                    pow(z-(center.z + params.offsetZ), 2)/(2*pow(params.sigmaZ, 2))));
            }
        }
    }
}


Point Grid::getWaypoint(){
    //for now, just return the max value
    Point maxPoint = center;

    for (int x = 0; x < grid->size(); x++){
        for (int y = 0; y < grid->size(); y++){
            for (int z = 0; z < grid->size(); z++){
                if ((*grid)[x][y][z] > (*grid)[maxPoint.x][maxPoint.y][maxPoint.z]){
                    maxPoint.x = x;
                    maxPoint.y = y;
                    maxPoint.z = z;
                }
            }
        }
    }
    return maxPoint;
}

void Grid::updateGrid(std::vector<Point> wallPoints){
    //adds inn the new wall points to the grid
    for (Point p : wallPoints){
        //TODO
        //assuming that the points are relative to the center, we have to adjust them
        if (p.x < 0 || p.y < 0 || p.z < 0 || p.x >= grid->size() || p.y >= grid->size() || p.z >= grid->size()){
            resizeGrid();
            //account for the new center
        }
        (*grid)[p.x][p.y][p.z] = -1;
    }
}

void Grid::resizeGrid(){
    //double size of the grid. memory limitations be dammned
    int newSize = grid->size()*2;
    std::unique_ptr<std::vector<std::vector<std::vector<double>>>> newGrid = 
        std::make_unique<std::vector<std::vector<std::vector<double>>>>
        (newSize, std::vector<std::vector<double>>(newSize, std::vector<double>(newSize, 0)));

    Point newCenter = {newSize/2, newSize/2, newSize/2};

    initGaussian(newGrid);

    //copy the old grid into the new grid, using center as the reference point
    for (int x = 0; x < grid->size(); x++){
        for (int y = 0; y < grid->size(); y++){
            for (int z = 0; z < grid->size(); z++){

                //is this the right way around?
                newGrid->at(x + center.x - newCenter.x).at(y + center.y - newCenter.y).at(z + center.z - newCenter.z) = grid->at(x).at(y).at(z);
            }
        }
    }
    grid = std::move(newGrid);
    center = newCenter;
}