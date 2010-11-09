#include "mvog_model/map.h"

namespace MVOG
{

Map::Map (double resolution, double sizeXmeters, double sizeYmeters)
{
  printf("%d {}\n", sizeof(Volume));

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

void Map::addPVolume(int x, int y, double top, double bottom)
{
  getCell(x,y)->addPVolume(top, bottom);
}

void Map::addNVolume(int x, int y, double top, double bottom)
{
  getCell(x,y)->addNVolume(top, bottom);
}


void Map::printPVolumes(int x, int y)
{
  getCell(x,y)->printPVolumes();
}

void Map::printNVolumes(int x, int y)
{
  getCell(x,y)->printNVolumes();
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
    {
      newGrid[i + sizeX_][j] = grid_[i][j];
      newGrid[i + sizeX_][j].pVolumes = new Volume[grid_[i][j].pVolumesCount];
      newGrid[i + sizeX_][j].nVolumes = new Volume[grid_[i][j].nVolumesCount];

      memcpy(newGrid[i + sizeX_][j].pVolumes, grid_[i][j].pVolumes, grid_[i][j].pVolumesCount * VOLUME_BYTE_SIZE);
      memcpy(newGrid[i + sizeX_][j].nVolumes, grid_[i][j].nVolumes, grid_[i][j].nVolumesCount * VOLUME_BYTE_SIZE);
    }
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
    {
      newGrid[i][j] = grid_[i][j];
      newGrid[i][j].pVolumes = new Volume[grid_[i][j].pVolumesCount];
      newGrid[i][j].nVolumes = new Volume[grid_[i][j].nVolumesCount];

      memcpy(newGrid[i][j].pVolumes, grid_[i][j].pVolumes, grid_[i][j].pVolumesCount * VOLUME_BYTE_SIZE);
      memcpy(newGrid[i][j].nVolumes, grid_[i][j].nVolumes, grid_[i][j].nVolumesCount * VOLUME_BYTE_SIZE);
    }

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
    {
      newGrid[i][j + sizeY_] = grid_[i][j];
      newGrid[i][j + sizeY_].pVolumes = new Volume[grid_[i][j].pVolumesCount];
      newGrid[i][j + sizeY_].nVolumes = new Volume[grid_[i][j].nVolumesCount];

      memcpy(newGrid[i][j + sizeY_].pVolumes, grid_[i][j].pVolumes, grid_[i][j].pVolumesCount * VOLUME_BYTE_SIZE);
      memcpy(newGrid[i][j + sizeY_].nVolumes, grid_[i][j].nVolumes, grid_[i][j].nVolumesCount * VOLUME_BYTE_SIZE);
    }
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
    {
      newGrid[i][j] = grid_[i][j];
      newGrid[i][j].pVolumes = new Volume[grid_[i][j].pVolumesCount];
      newGrid[i][j].nVolumes = new Volume[grid_[i][j].nVolumesCount];

      memcpy(newGrid[i][j].pVolumes, grid_[i][j].pVolumes, grid_[i][j].pVolumesCount * VOLUME_BYTE_SIZE);
      memcpy(newGrid[i][j].nVolumes, grid_[i][j].nVolumes, grid_[i][j].nVolumesCount * VOLUME_BYTE_SIZE);
    }

    // delete old grid
    deleteGrid();

    // change size variables
    sizeY_ *= 2;
    grid_ = newGrid;

    return getCell(x, y);
  }

  return &grid_[cx][cy];
}

double Map::getMemorySize()
{
    double gridSize = sizeX_ * sizeY_ * sizeof(Cell);

    double dataSize = 0;
    for (int i = 0; i< sizeX_; ++i)
    for (int j = 0; j< sizeY_; ++j)
      dataSize += (grid_[i][j].getPVolumesCount() + grid_[i][j].getPVolumesCount()) * VOLUME_BYTE_SIZE;

    return (gridSize + dataSize)/1024.0;
}

void Map::test()
{
  addPVolume(0, 0, 9.0, 10.4);
  addPVolume(0, 0, 7.0, 8.3);
  addPVolume(0, 0, 5.0, 6.3);
  addPVolume(0, 0, 3.0, 4.2);
  addPVolume(0, 0, 1.0, 2.1);
  addPVolume(0, 0, -1.0, -0.0);
  addPVolume(0, 0, -3.0, -2.0);

  printPVolumes(0, 0);

  addNVolume(0, 0, 0, 5.0);

  printNVolumes(0, 0);

  MLVolumeVector m;
  getCell(0,0)->createMLVolumes(m);
}

}
