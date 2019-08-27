#include "../include/quadtree_planner/costmap.h"

namespace quadtree_planner {

    EmptyCostmap::EmptyCostmap(unsigned int sizeX, unsigned int sizeY, double resolution)
            : sizeX(sizeX), sizeY(sizeY), resolution(resolution) {}

    unsigned int EmptyCostmap::getCost(unsigned int x, unsigned int y) const {
        return 0;
    }

    double EmptyCostmap::getSizeInMetersX() const {
        return sizeX * resolution;
    }

    double EmptyCostmap::getSizeInMetersY() const {
        return sizeY * resolution;
    }

    unsigned int EmptyCostmap::getSizeInCellsX() const {
        return sizeX;
    }

    unsigned int EmptyCostmap::getSizeInCellsY() const {
        return sizeY;
    }

    double EmptyCostmap::getResolution() const {
        return resolution;
    }

    bool EmptyCostmap::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
        uint x = uint(wx / resolution);
        uint y = uint(wy / resolution);
        if (x < 0 || x >= getSizeInCellsX() || y < 0 || y >= getSizeInCellsY()) {
            return false;
        }
        mx = x;
        my = y;
        return true;
    }

    EmptyCostmap::~EmptyCostmap() = default;

    CostmapAdapter::CostmapAdapter(costmap_2d::Costmap2D *costmap): costmap_(costmap) {}

    unsigned int CostmapAdapter::getCost(unsigned int x, unsigned int y) const {
        return costmap_->getCost(x, y);
    }

    double CostmapAdapter::getSizeInMetersX() const {
        return costmap_->getSizeInMetersX();
    }
    double CostmapAdapter::getSizeInMetersY() const {
        return costmap_->getSizeInMetersY();
    }

    uint CostmapAdapter::getSizeInCellsX() const {
        return costmap_->getSizeInCellsX();
    }

    uint CostmapAdapter::getSizeInCellsY() const {
        return costmap_->getSizeInCellsY();
    }

    double CostmapAdapter::getResolution() const {
        return costmap_->getResolution();
    }

    void CostmapAdapter::setCost(unsigned int mx, unsigned int my, unsigned char cost) const {
        costmap_->setCost(mx, my, cost);
    }

    bool CostmapAdapter::saveMap(std::string file_name) const {
        costmap_->saveMap(file_name);
    }

    bool CostmapAdapter::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const {
        return costmap_->worldToMap(wx, wy, mx, my);
    }

    // Cell based search
    void CostmapAdapter::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const {
        costmap_->mapToWorld(mx, my, wx, wy);
        return;
    }

    CostmapAdapter::~CostmapAdapter() = default;
}
