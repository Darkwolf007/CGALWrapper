#include "pch.h"
#include "CGALWrapper.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/partition_2.h>
#include <CGAL/Polygon_2.h>

#include <vector>
#include <list>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;

typedef K::Point_2 Point_2;
typedef std::list<Point_2> PointList;
typedef CGAL::Polygon_2<K, PointList> Polygon_2;

extern "C" __declspec(dllexport) void ComputeOBB(double* points, int count, double* obbCorners)
{
    if (count < 3 || points == nullptr || obbCorners == nullptr)
        return;

    std::vector<Point> pointVec;
    pointVec.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        double x = points[i * 3 + 0];
        double y = points[i * 3 + 1];
        double z = points[i * 3 + 2];
        pointVec.emplace_back(x, y, z);
    }

    std::array<Point, 8> obbPoints;

    CGAL::oriented_bounding_box(pointVec, obbPoints, CGAL::parameters::use_convex_hull(true));

    for (int i = 0; i < 8; ++i)
    {
        obbCorners[i * 3 + 0] = obbPoints[i].x();
        obbCorners[i * 3 + 1] = obbPoints[i].y();
        obbCorners[i * 3 + 2] = obbPoints[i].z();
    }
}

extern "C" __declspec(dllexport) int PartitionPolygonApprox(
    double* inputXY, int count, double* outputXY, int* polygonSizes, int maxPolygons)
{
    if (count < 3 || inputXY == nullptr || outputXY == nullptr || polygonSizes == nullptr)
        return 0;

    std::vector<Point_2> inputPoints;
    inputPoints.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        double x = inputXY[i * 2 + 0];
        double y = inputXY[i * 2 + 1];
        inputPoints.emplace_back(x, y);
    }

    Polygon_2 polygon(inputPoints.begin(), inputPoints.end());
    if (!polygon.is_simple() || polygon.orientation() != CGAL::COUNTERCLOCKWISE)
        polygon.reverse_orientation();

    std::list<Polygon_2> partitionPolygons;

    // Use convex partitioning (can be replaced with y_monotone_partition_2 or optimal_convex_partition_2)
    CGAL::approx_convex_partition_2(
        polygon.vertices_begin(),
        polygon.vertices_end(),
        std::back_inserter(partitionPolygons)
    );

    int index = 0;
    int partIndex = 0;
    for (const auto& part : partitionPolygons)
    {
        if (partIndex >= maxPolygons) break;

        polygonSizes[partIndex] = static_cast<int>(part.size());

        for (const auto& p : part)
        {
            outputXY[index++] = p.x();
            outputXY[index++] = p.y();
        }

        ++partIndex;
    }

    return partIndex;
}


extern "C" __declspec(dllexport) int PartitionPolygonOptimal(
    double* inputXY, int count, double* outputXY, int* polygonSizes, int maxPolygons)
{
    if (count < 3 || inputXY == nullptr || outputXY == nullptr || polygonSizes == nullptr)
        return 0;

    std::vector<Point_2> inputPoints;
    inputPoints.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        double x = inputXY[i * 2 + 0];
        double y = inputXY[i * 2 + 1];
        inputPoints.emplace_back(x, y);
    }

    Polygon_2 polygon(inputPoints.begin(), inputPoints.end());
    if (!polygon.is_simple() || polygon.orientation() != CGAL::COUNTERCLOCKWISE)
        polygon.reverse_orientation();

    std::list<Polygon_2> partitionPolygons;

    // Use convex partitioning (can be replaced with y_monotone_partition_2 or optimal_convex_partition_2)
    CGAL::optimal_convex_partition_2(
        polygon.vertices_begin(),
        polygon.vertices_end(),
        std::back_inserter(partitionPolygons)
    );

    int index = 0;
    int partIndex = 0;
    for (const auto& part : partitionPolygons)
    {
        if (partIndex >= maxPolygons) break;

        polygonSizes[partIndex] = static_cast<int>(part.size());

        for (const auto& p : part)
        {
            outputXY[index++] = p.x();
            outputXY[index++] = p.y();
        }

        ++partIndex;
    }

    return partIndex;
}

extern "C" __declspec(dllexport) int PartitionPolygonMonotone(
    double* inputXY, int count, double* outputXY, int* polygonSizes, int maxPolygons)
{
    if (count < 3 || inputXY == nullptr || outputXY == nullptr || polygonSizes == nullptr)
        return 0;

    std::vector<Point_2> inputPoints;
    inputPoints.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        double x = inputXY[i * 2 + 0];
        double y = inputXY[i * 2 + 1];
        inputPoints.emplace_back(x, y);
    }

    Polygon_2 polygon(inputPoints.begin(), inputPoints.end());
    if (!polygon.is_simple() || polygon.orientation() != CGAL::COUNTERCLOCKWISE)
        polygon.reverse_orientation();

    std::list<Polygon_2> partitionPolygons;

    // Use convex partitioning (can be replaced with y_monotone_partition_2 or optimal_convex_partition_2)
    CGAL::y_monotone_partition_2(
        polygon.vertices_begin(),
        polygon.vertices_end(),
        std::back_inserter(partitionPolygons)
    );

    int index = 0;
    int partIndex = 0;
    for (const auto& part : partitionPolygons)
    {
        if (partIndex >= maxPolygons) break;

        polygonSizes[partIndex] = static_cast<int>(part.size());

        for (const auto& p : part)
        {
            outputXY[index++] = p.x();
            outputXY[index++] = p.y();
        }

        ++partIndex;
    }

    return partIndex;
}


