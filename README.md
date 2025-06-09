# CGAL for Grasshopper —  Computational Geometry 

Bring the power of CGAL's computational geometry algorithms into Rhino Grasshopper with a clean C++ DLL and .NET P/invovke bridge.  
This project exposes optimized geometry operations like Oriented Bounding Boxes, Convex Partitioning, and Straight Skeletons to designers, computational architects, and developers working in Grasshopper.

---

## Features

- **Optimal Oriented Bounding Box** (CGAL::oriented_bounding_box)
- **Polygon Partitioning**
  - Approximate Convex Partition
  - Optimal Convex Partition
  - Y-Monotone Partition
- **2D Straight Skeleton**
- Designed for **real-time use** inside Rhino Grasshopper via P/Invoke
- Easily extendable with more CGAL modules

---

## Prerequisites

- **Visual Studio 2019/2022**
- **vcpkg** (installed and integrated)
- **Rhino 7 or 8**
- Basic familiarity with C++ and C# (for P/Invoke usage in Grasshopper)

---

## Clone & Setup

```bash
git clone https://github.com/yourusername/cgal-grasshopper-integration.git
cd cgal-grasshopper-integration
