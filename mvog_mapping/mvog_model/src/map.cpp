#include "mvog_model/map.h"

namespace MVOG
{

Map::Map (double resolution, double sizeXmeters, double sizeYmeters)
{
  resolution_ = resolution;

  sizeX_ = sizeXmeters / resolution_ + 1;
  sizeY_ = sizeYmeters / resolution_ + 1;
  
  printf("Map size:  %f x %f meters\n", sizeXmeters, sizeYmeters);
  printf("Grid size: %d x %d cells\n", sizeX_, sizeY_);

  offsetX_ = sizeX_ / 2;
  offsetY_ = sizeY_ / 2;

  grid_ = initializeGrid(sizeX_, sizeY_);
}

Map::~Map ()
{


}

void Map::addVolume(int x, int y, double top, double bottom)
{
  grid_[x][y].addPVolume(top, bottom);
}

void Map::printVolumes(int x, int y)
{
  grid_[x][y].printPVolumes();
}

void Map::deleteGrid()
{
   for (int i = 0; i < sizeX_; i++) delete[] grid_[i];
   delete[] grid_;
}

Cell** Map::initializeGrid(int sizeX, int sizeY)
{
  Cell** grid;

  grid = new Cell*[sizeX];
  for (int i = 0; i < sizeX; i++)
    grid[i] = new Cell[sizeY];

  return grid;
}

Cell* Map::getCell(double x, double y)
{
  int cx = floor(x) + offsetX_;
  int cy = floor(y) + offsetY_;

  if (cx < 0)
  { 
    // initialize new grid
    MVOG::Cell** newGrid = initializeGrid(sizeX_*2, sizeY_);

    // copy information over
    for (int i = 0; i < sizeX_; i++)
    for (int j = 0; j < sizeY_; j++)
      newGrid[i + sizeX_][j] = grid_[i][j];

    // delete old grid
    deleteGrid();

    // change size variables
    offsetX_ += sizeX_;
    sizeX_ *= 2;
    grid_ = newGrid;

    return getCell(x, y);
  }
  if (cx >= sizeX_)
  { 
    // initialize new grid
    MVOG::Cell** newGrid = initializeGrid(sizeX_*2, sizeY_);

    // copy information over
    for (int i = 0; i < sizeX_; i++)
    for (int j = 0; j < sizeY_; j++)
      newGrid[i][j] = grid_[i][j];

    // delete old grid
    deleteGrid();

    // change size variables
    sizeX_ *= 2;
    grid_ = newGrid;

    return getCell(x, y);
  }
  if (cy < 0)
  { 
    // initialize new grid
    MVOG::Cell** newGrid = initializeGrid(sizeX_, sizeY_*2);

    // copy information over
    for (int i = 0; i < sizeX_; i++)
    for (int j = 0; j < sizeY_; j++)
      newGrid[i][j + sizeY_] = grid_[i][j];

    // delete old grid
    deleteGrid();

    // change size variables
    offsetY_ += sizeY_;
    sizeY_ *= 2;
    grid_ = newGrid;

    return getCell(x, y);
  }
  if (cy >= sizeY_)
  { 
    // initialize new grid
    MVOG::Cell** newGrid = initializeGrid(sizeX_, sizeY_*2);

    // copy information over
    for (int i = 0; i < sizeX_; i++)
    for (int j = 0; j < sizeY_; j++)
      newGrid[i][j] = grid_[i][j];

    // delete old grid
    deleteGrid();

    // change size variables
    sizeY_ *= 2;
    grid_ = newGrid;

    return getCell(x, y);
  }

  return &grid_[cx][cy];
}

}
