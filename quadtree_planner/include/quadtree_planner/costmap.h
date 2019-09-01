//
// Created by alex on 16.02.19.
//

#ifndef ASTAR_PLANNER_COSTMAP_H
#define ASTAR_PLANNER_COSTMAP_H

#include <costmap_2d/costmap_2d.h>

namespace quadtree_planner {

    /**
     * Associates costs with the points in the world.
     * Largely corresponds to costmap_2d/Costmap2D of the ROS navigation stack.
     */
    class Costmap {
    public:
        /**
         * @brief  Get the cost of a cell in the costmap
         * @param mx The x coordinate of the cell
         * @param my The y coordinate of the cell
         * @return The cost of the cell
         */
        virtual unsigned getCost(unsigned int x, unsigned int y) const = 0;

        /**
         * @brief  Get the size of the costmap in x direction in meters
         * @return size of the costmap in x direction in meters
         */
        virtual double getSizeInMetersX() const = 0;

        /**
        * @brief  Get the size of the costmap in y direction in meters
        * @return size of the costmap in y direction in meters
        */
        virtual double getSizeInMetersY() const = 0;

        /**
        * @brief  Get the size of the costmap in x direction in cells
        * @return size of the costmap in x direction in cells
        */
        virtual unsigned int getSizeInCellsX() const = 0;

        /**
        * @brief  Get the size of the costmap in y direction in cells
        * @return size of the costmap in y direction in cells
        */
        virtual unsigned int getSizeInCellsY() const = 0;

        /**
        * @brief  Set the cost of a cell in the costmap
        * @param mx The x coordinate of the cell
        * @param my The y coordinate of the cell
        */
        virtual void setCost(unsigned int mx, unsigned int my, unsigned char cost) const = 0;

        /**
         * @brief  Get resolution of the costmap, in meters per cell.
         * @return  resolution of the costmap, in meters per cell
         */
        virtual double getResolution() const = 0;

        /**
          * @brief  Convert from world coordinates to map coordinates
          * @param  wx The x world coordinate
          * @param  wy The y world coordinate
          * @param  mx Will be set to the associated map x coordinate
          * @param  my Will be set to the associated map y coordinate
          * @return True if the conversion was successful (legal bounds) false otherwise
          */
        virtual bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const = 0;

        /**
          * @brief  Convert from map coordinates to world coordinates
          * @param  mx The x map coordinate
          * @param  my The y map coordinate
          * @param  wx Will be set to the associated world x coordinate
          * @param  wy Will be set to the associated world y coordinate
          */
        virtual void mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const = 0;

        /**
         * @brief the costmap to a file as a greyscale image
         * @param file_name path to file where the image shall be stored (e.g. as .pgm)
         * @return
         */
        virtual bool saveMap(std::string file_name) const = 0;

        /**
         * @brief the origin of the map in x direction in meters
         * @return origin of the map in x direction in meters
         */
        virtual double getOriginX() const = 0;

        /**
        * @brief the origin of the map in y direction in meters
        * @return origin of the map in y direction in meters
        */
        virtual double getOriginY() const = 0;

        virtual ~Costmap() = default;
    };

    /**
     * A stub costmap used for testing purposes.
     */
    class EmptyCostmap : public Costmap {
    private:
        unsigned int sizeX;
        unsigned int sizeY;
        double resolution;
    public:

        /**
         * @brief Constructor for EmptyCostmap class
         * @param sizeX size of the costmap in cells in x direction
         * @param sizeY size of the costmap in cells in y direction
         * @param resolution resolution of the costmap, in meters per cell
         */
        EmptyCostmap(unsigned int sizeX, unsigned int sizeY, double resolution);

        /**
        * @brief  Get the cost of a cell in the costmap
        * @param mx The x coordinate of the cell
        * @param my The y coordinate of the cell
        * @return The cost of the cell
        */
        unsigned int getCost(unsigned int x, unsigned int y) const override;

        /**
        * @brief  Get the size of the costmap in x direction in meters
        * @return size of the costmap in x direction in meters
        */
        double getSizeInMetersX() const override;

        /**
        * @brief  Get the size of the costmap in y direction in meters
        * @return size of the costmap in y direction in meters
        */
        double getSizeInMetersY() const override;

        /**
        * @brief  Get the size of the costmap in x direction in cells
        * @return size of the costmap in x direction in cells
        */
        unsigned int getSizeInCellsX() const override;

        /**
        * @brief  Get the size of the costmap in y direction in cells
        * @return size of the costmap in y direction in cells
        */
        unsigned int getSizeInCellsY() const override;

        /**
        * @brief  Get resolution of the costmap, in meters per cell.
        * @return  resolution of the costmap, in meters per cell
        */
        double getResolution() const override;

        /**
        * @brief  Convert from world coordinates to map coordinates
        * @param  wx The x world coordinate
        * @param  wy The y world coordinate
        * @param  mx Will be set to the associated map x coordinate
        * @param  my Will be set to the associated map y coordinate
        * @return True if the conversion was successful (legal bounds) false otherwise
        */
        bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const override;

        ~EmptyCostmap() override;
    };

    /**
     * An adapter that fullfils the Costmap interfaces and delegates to a costmap_2d/Costmap2D impolemenation.
     */
    class CostmapAdapter : public Costmap {
    private:
        costmap_2d::Costmap2D *costmap_;
    public:
        /**
         * @brief  Constructor for CostmapAdapter class
         * @param costmap  pointer to costmap_2d ros object instance
         */
        CostmapAdapter(costmap_2d::Costmap2D *costmap);

        /**
        * @brief  Get the cost of a cell in the costmap
        * @param mx The x coordinate of the cell
        * @param my The y coordinate of the cell
        * @return The cost of the cell
        */
        unsigned int getCost(unsigned int x, unsigned int y) const override;

        /**
        * @brief  Get the size of the costmap in x direction in meters
        * @return size of the costmap in x direction in meters
        */
        double getSizeInMetersX() const override;

        /**
        * @brief  Get the size of the costmap in y direction in meters
        * @return size of the costmap in y direction in meters
        */
        double getSizeInMetersY() const override;

        /**
        * @brief  Get the size of the costmap in x direction in cells
        * @return size of the costmap in x direction in cells
        */
        unsigned int getSizeInCellsX() const override;

        /**
        * @brief  Get the size of the costmap in y direction in cells
        * @return size of the costmap in y direction in cells
        */
        unsigned int getSizeInCellsY() const override;

        /**
        * @brief  Get resolution of the costmap, in meters per cell.
        * @return  resolution of the costmap, in meters per cell
        */
        double getResolution() const override;

        /**
        * @brief  Set the cost of a cell in the costmap
        * @param mx The x coordinate of the cell
        * @param my The y coordinate of the cell
        */
        void setCost(unsigned int mx, unsigned int my, unsigned char cost) const override;

        /**
        * @brief  Convert from world coordinates to map coordinates
        * @param  wx The x world coordinate
        * @param  wy The y world coordinate
        * @param  mx Will be set to the associated map x coordinate
        * @param  my Will be set to the associated map y coordinate
        * @return True if the conversion was successful (legal bounds) false otherwise
        */
        bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const override;

        /**
        * @brief  Convert from map coordinates to world coordinates
        * @param  mx The x map coordinate
        * @param  my The y map coordinate
        * @param  wx Will be set to the associated world x coordinate
        * @param  wy Will be set to the associated world y coordinate
        */
        void mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const override;

        /**
        * @brief the costmap to a file as a greyscale image
        * @param file_name path to file where the image shall be stored (e.g. as .pgm)
        * @return
        */
        bool saveMap(std::string file_name) const override;

        /**
        * @brief the origin of the map in x direction in meters
        * @return origin of the map in x direction in meters
        */
        double getOriginX() const override;

        /**
        * @brief the origin of the map in y direction in meters
        * @return origin of the map in y direction in meters
        */
        double getOriginY() const override;

        ~CostmapAdapter() override;
    };
}
#endif //ASTAR_PLANNER_COSTMAP_H
