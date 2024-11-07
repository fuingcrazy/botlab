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

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan
    
    // Hint: Consider both the cells the laser hit and the cells it passed through.
   int size = movingScan.size();
   auto it = movingScan.begin();
   while(it != movingScan.end()){
    scoreEndpoint(*it,map);
    scoreRay(*it,map);
    it ++;
   }
    

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits
    int startx = ray.origin.x*map.cellsPerMeter();
    int starty = ray.origin.y*map.cellsPerMeter();
    int endx = startx + ray.range*cos(ray.theta)*map.cellsPerMeter();
    int endy = starty + ray.range*sin(ray.theta)*map.cellsPerMeter();
    map.setLogOdds(endx,endy,map.logOdds(endx,endy)+kHitOdds_);
    std::cout<<"Hit position:"<<endx<<", "<<endy<<". LogOdds: "<<map.logOdds(endx,endy)+kHitOdds_<<std::endl;

}

void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through
   auto it = bresenham(ray,map).begin();
   auto end = bresenham(ray,map).end();
   while (it != end ){
      map.setLogOdds(it->x,it->y,map.logOdds(it->x,it->y)-kMissOdds_);
      it ++;
      std::cout<<"Missed position: "<<it->x<<", "<<it->y<<".LogOdds: "<<map.logOdds(it->x,it->y)-kMissOdds_<<std::endl;
   }
    
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    std::vector<Point<int>> points;
    int dx = fabs(ray.range*cos(ray.theta)*map.cellsPerMeter());
    int dy = fabs(ray.range*sin(ray.theta)*map.cellsPerMeter());
    int sx = (dx>0) ? 1:-1;
    int sy = (dy>0) ? 1:-1;
    int err = dx - dy;
    int x = ray.origin.x*map.cellsPerMeter(),y = ray.origin.y*map.cellsPerMeter();
    int endx = x + ray.range*cos(ray.theta)*map.cellsPerMeter(),endy = ray.range*sin(ray.theta)*map.cellsPerMeter();
    tempPoint = std::make_unique<Point<int>>(x,y);
    points.push_back(*tempPoint);
    while(x != endx || y != endy){
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
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Decrease the odds of the cell at (x,y)
}
