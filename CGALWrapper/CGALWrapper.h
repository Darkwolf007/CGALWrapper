#pragma once

#ifdef CGALWRAPPER_EXPORTS
#define CGALWRAPPER_API __declspec(dllexport)
#else
#define CGALWRAPPER_API __declspec(dllimport)
#endif

extern "C" {
    // Existing OBB function
    CGALWRAPPER_API void ComputeOBB(double* points, int count, double* obbCorners);

    // New 2D Polygon Partitioning function
    // - inputXY: flat 2D coords [x0, y0, x1, y1, ..., xn, yn]
    // - count: number of input points
    // - outputXY: flattened array of resulting polygon vertices
    // - polygonSizes: number of vertices per polygon
    // - maxPolygons: size of polygonSizes array
    // Returns number of resulting polygons
    CGALWRAPPER_API int PartitionPolygonApprox(double* inputXY, int count, double* outputXY, int* polygonSizes, int maxPolygons);
    CGALWRAPPER_API int PartitionPolygonOptimal(double* inputXY, int count, double* outputXY, int* polygonSizes, int maxPolygons);
    CGALWRAPPER_API int PartitionPolygonMonotone(double* inputXY, int count, double* outputXY, int* polygonSizes, int maxPolygons);

    CGALWRAPPER_API int ComputeSkeleton(double* inputXY, int count, double* outputXY, int* edgePairs, int maxEdges);
}
