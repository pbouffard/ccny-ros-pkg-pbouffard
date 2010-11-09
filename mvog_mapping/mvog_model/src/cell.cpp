#include "mvog_model/cell.h"

namespace MVOG
{

Cell::Cell()
{
  pVolumesCount = 0;
  nVolumesCount = 0;
  pVolumes = NULL;
  nVolumes = NULL;
}

Cell::~Cell()
{
  delete[] pVolumes;
  delete[] nVolumes;
  
  pVolumes = NULL;
  nVolumes = NULL;
}

void Cell::addPVolume(float bot, float top)
{
	addVolume(bot, top, pVolumes, pVolumesCount);
}

void Cell::addNVolume(float bot, float top)
{
	addVolume(bot, top, nVolumes, nVolumesCount);
}

void Cell::addVolume(float bot, float top, VolumeArray& volumes, int& volumesCount)
{
  float minGap = 0.0; // FIXME: gap only works for 0.0, - gap of 1.0 needs code for adj. mass

  // **** create the volume
  Volume v;
  createVolume(bot, top, v);

  // **** non-empty list: iterate over volumes
  for (int i = 0; i < volumesCount; i++)
  {
    // 1. below next volume, and does not intersect
    if (getTop(v) < getBot(volumes[i]) - minGap)
    {
      // insert it at position i;
      VolumeArray newVolumes = new Volume[volumesCount+1];
      memcpy(newVolumes, volumes, i*VOLUME_BYTE_SIZE);
      memcpy(newVolumes[i], v, VOLUME_BYTE_SIZE);
      memcpy(newVolumes+i+1, volumes, (volumesCount-i)*VOLUME_BYTE_SIZE);
      delete[] volumes;
      volumes = newVolumes;  
      volumesCount++;   

      return;
    }
    // 2. intersects with next volume
    else if (getBot(v) <= getTop(volumes[i]) + minGap)
    {
      // get all volumes that v intersects with
      int stopIndex  = i+1;

      int c = i+1;
      while(c < volumesCount )
      {
        if (getTop(v) >= getBot(volumes[c]) - minGap)
        {
           c++;
           stopIndex = c;
        }
        else break;
      }

      // merge with new volume

      setBot (volumes[i], std::min(getBot(volumes[i]), getBot(v)));
      setTop (volumes[i], std::max(getTop(volumes[i]), getTop(v)));
      setMass(volumes[i], getMass(volumes[i]) + getMass(v)); // + FIXME: gap mass??

      // merge with rest
      setTop(volumes[i], std::max(getTop(volumes[i]), getTop(volumes[stopIndex - 1])));

      for (int t = i+1; t < stopIndex ; t++)
         setMass(volumes[i], getMass(volumes[i]) + getMass(volumes[t])); // + FIXME: gap mass??

      // erase old volumes that were merged
      //printf ("erasing from %d to %d\n", i+1, stopIndex);
      
      VolumeArray newVolumes = new Volume[volumesCount - (stopIndex - i) + 1];
      memcpy(newVolumes+0, volumes, (i+1)*VOLUME_BYTE_SIZE);
      memcpy(newVolumes+i+1, volumes + stopIndex, (volumesCount - stopIndex)*VOLUME_BYTE_SIZE);
      delete[] volumes; 
      volumes = newVolumes;
      volumesCount = volumesCount - (stopIndex - i) + 1;

      return;
    }
  }

  // insert it at position i;

  VolumeArray newVolumes = new Volume[volumesCount+1];
  memcpy(newVolumes, volumes, volumesCount*VOLUME_BYTE_SIZE);
  memcpy(newVolumes[volumesCount], v, VOLUME_BYTE_SIZE);
  delete[] volumes;
  volumes = newVolumes;

  volumesCount++;
}

void Cell::printPVolumes()
{
  printf("*** CELL (+) %d ****\n", pVolumesCount);

  for (int i = 0; i < pVolumesCount; i++)
    printf("\t%d [%f, %f] [%f]\n", i, getBot(pVolumes[i]), getTop(pVolumes[i]), getMass(pVolumes[i]));
}

void Cell::printNVolumes()
{
  printf("*** CELL (-) %d ****\n", nVolumesCount);

  for (int i = 0; i < nVolumesCount; i++)
    printf("\t%d [%f, %f] [%f]\n", i, getBot(nVolumes[i]), getTop(nVolumes[i]), getMass(nVolumes[i]));
}


bool Cell::getOccDensity(float z, float& density) const
{
  float pd = getPDensity(z);
  float nd = getNDensity(z);

  if (pd == 0 && nd == 0) return false;

  density = pd / (pd + nd);

  return true;
}

float Cell::getPDensity(float z) const
{
  for(int i = 0; i < pVolumesCount; ++i)
  {
    if (getBot(pVolumes[i]) <= z && z <= getTop(pVolumes[i]))
      return getDensity(pVolumes[i]);
    if (z < getBot(pVolumes[i]))
      return 0;
  }
  
	return 0;
}

float Cell::getNDensity(float z) const
{
  for(int i = 0; i < nVolumesCount; ++i)
  {
    if (getBot(nVolumes[i]) <= z && z <= getTop(nVolumes[i]))
      return getDensity(nVolumes[i]);
    if (z < getBot(nVolumes[i]))
      return 0;
  }
  
	return 0;
}

void Cell::createMLVolumes(MLVolumeVector& mlVolumes)
{
  // **** convert volumes to boundary points

  std::vector<Boundary> pBoundaries;
  std::vector<Boundary> nBoundaries;

  std::priority_queue<Boundary> q;

  for (int i = 0; i < pVolumesCount; i++)
  {
    Boundary b1(getBot(pVolumes[i]), true, i);
    Boundary b2(getTop(pVolumes[i]), true, i);
    q.push(b1);
    q.push(b2);
  }

  for (int i = 0; i < nVolumesCount; i++)
  {
    Boundary b1(getBot(nVolumes[i]), false, i);
    Boundary b2(getTop(nVolumes[i]), false, i);
    q.push(b1);
    q.push(b2);
  }

/*
  printf("---- BOUNDARIES: ----\n");

  while(!q.empty())
  {
    Boundary b = q.top();
    q.pop();

    if( b.positive_) printf("(+) ");
    else             printf("(-) ");
  
    printf("%f [%d]\n", b.z_, b.index_);
  }
*/
  
  float p_pd = 0;
  float p_nd = 0;

  float p_pd = 0;
  float p_nd = 0;

  MLVolume vol;
  MLVolumeVector vect;
  Boundary  boundary;

  if (!q.empty())
  {
    boundary = q.top();
    q.pop();

    if (boundary.positive_) p_pd = getDensity(pVolumes[boundary.index_]);
    else                       p_nd = getDensity(nVolumes[boundary.index_]);
 
    while(!q.empty())
    {
      boundary = q.top();
      q.pop();

      float c_pd;
      float c_nd;
      
      if (boundary.positive_)
      {
        float c_pd = getDensity(pVolumes[boundary.index_]);
        float c_nd = p_nd;

        if (p_pd == 0) // entering a positive volume
        {
          if (c_pd/(c_pd + c_nd) >= 0.5) vol.bot = getBot(pVolumes[boundary.index_]);

        }
      }
      else
      {

      }


      p_pd = c_pd;
      p_nd = c_nd;
      botBoundary = topBoundary;
    }
  }

}

bool isObstacle(int pIndex, int nIndex)
{


}


}
