#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
    : kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds), initialized_(false)
{
}

void Mapping::updateMap(const mbot_lcm_msgs::lidar_t &scan,
                        const mbot_lcm_msgs::pose2D_t &pose,
                        OccupancyGrid &map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;
    //std::cout<<"KhitOdds: "<<kHitOdds_<<", KMissOdds: "<<kMissOdds_<<std::endl;
    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan
    
    // Hint: Consider both the cells the laser hit and the cells it passed through.
   int size = movingScan.size();
   auto it = movingScan.begin();
   //std::cout<<"size: "<<size<<std::endl;
   while(it != movingScan.end()){
    scoreEndpoint(*it,map);
    scoreRay(*it,map);
    it ++;
   }
    //  for(std::size_t y = 0; y < map.heightInCells(); ++y)
    //    {
    //        for(std::size_t x = 0; x < map.widthInCells(); ++x)
    //        {
    //            std::cout<<map.logOdds(x,y)<<" ,";
    //        }
    //        std::cout<<std::endl;
    //    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits
    //std::cout<<"Origin: "<<ray.origin.x<<std::endl;
    if(ray.range < kMaxLaserDistance_)
    {
    Point<float> startPt = global_position_to_grid_position(ray.origin,map);
    int endx = startPt.x + ray.range*cos(ray.theta)*map.cellsPerMeter();
    int endy = startPt.y + ray.range*sin(ray.theta)*map.cellsPerMeter();
    increaseCellOdds(endx,endy,map);
    //map.setLogOdds(endx,endy,map.logOdds(endx,endy)+kHitOdds_);
    std::cout<<"Hit position:"<<endx<<", "<<endy<<". LogOdds: "<<map.logOdds(endx,endy)+kHitOdds_<<std::endl;
    }
}

void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through
  if(ray.range < kMaxLaserDistance_){
   Point<float> startPt = global_position_to_grid_position(ray.origin,map);
   int endx = startPt.x + ray.range*cos(ray.theta)*map.cellsPerMeter();
   int endy = startPt.y + ray.range*sin(ray.theta)*map.cellsPerMeter();
   auto it = bresenham(startPt.x,startPt.y,endx,endy,map).begin();
   auto end = bresenham(startPt.x,startPt.y,endx,endy,map).end();
   //std::cout<<"End position: "<<end->x<<", "<<end->y<<std::endl;
   while (it != end ){
      decreaseCellOdds(it->x,it->y,map);
      //map.setLogOdds(it->x,it->y,map.logOdds(it->x,it->y) - kMissOdds_);
      it ++;
  }
  std::cout<<"decrease done."<<std::endl;
  }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(int x1,int y1,int x2, int y2, const OccupancyGrid &map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    std::vector<Point<int>> points;
    int dx = fabs(x1-x2);
    int dy = fabs(y1-y2);
    int sx = (x1<x2) ? 1:-1;
    int sy = (y1<y2) ? 1:-1;
    int err = dx - dy;
    tempPoint = std::make_unique<Point<int>>(x1,y1);
    points.push_back(*tempPoint);
    int x = x1,y = y1;
   // std::cout<<"Info:dx: "<<dx<<", dy:"<<dy<<" endx:"<<endx<<", endy:"<<endy<<" ,x:"<<x<<" ,y:"<<y<<" .sx:"<<sx<<", sy: "<<sy<<std::endl;

    while(x != x2 || y != y2){
        int e2 = 2 * err;
        if(e2 >= -dy){
            err -= dy;
            x += sx;
        }
        if(e2 <= dx){
            err += dx;
            y += sy;
        }
         tempPoint = std::make_unique<Point<int>>(x,y);
         //std::cout<<" x: "<<x<<", y: "<<y<<std::endl;
         points.push_back(*tempPoint);
    }

    return points; // Placeholder
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray.
    return {}; // Placeholder
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Increase the odds of the cell at (x,y)
    if(map.isCellInGrid(x,y)){
    if(INT8_MAX - map(x,y) > kHitOdds_)
       map(x,y) += kHitOdds_;

    else
       map(x,y) = INT8_MAX;
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Decrease the odds of the cell at (x,y)
    if(map.isCellInGrid(x,y)){
    if(map(x,y) - kMissOdds_ >INT8_MIN)
       map(x,y) -= kMissOdds_;
    else
       map(x,y) = INT8_MIN;
    }
}

