# Constrained Delaunay Triangulation
C++ Implementation of the Constrained Delaunay Triangulation (CDT) made specifically for games, that use a rectangular shaped map. The triangulation vertices lie on positive integer indexed **Grid**, which is ideal for most grid based games. This means we don't have to rely on floating point arithmetic in order to check if a vertex exists in the CDT. The user will be allowed to pick a coordinate type as either 8bit, 16bit, or 32 bit unsigned integer depending. The advantage of using smaller types is that triangles themselves are smaller and thus more of them fit into a single cache line.


The CDT could be used how the CDT can be used to implement:
1. pathfinding
2. shadow casting and vision field construction
3. surface extraction from points in 3D (e.g. for terrain generation)
4. ???
5. profit 

## Build

the library can be built using CMake. From your project directory do:

```
mkdir build
cd build
cmake ../
```

If you want to see Demo use the ``-DBUILD_EXAMPLE=ON`` flag when calling cmake. For building tests use:
``-DBUILD_TESTS=ON``

## Usage

To create a CDT object contained in a given 200x100 rectangle: 
```
Triangulation<Triangle> cdt({200, 100});
```
Inserting vertices can be done using the `VertexData insertVertexAndGetData(Vertex pos)` method. The method returns an object containing information about whether the inserted vertex:
1. was inserted into empty space. In this case the `VertexData` object contains vertex index value of -1
2. already existed in the CDT. In this cas the `VertexData` object contains the index of the existing vertex
3. was inserted on an existing constrained edge. In this case the `VertexData` object contains edge data of the edge

```
auto v_data = cdt.insertVertexAndGetData(69, 69);
assert(v_data2.overlapping_vertex == -1)

auto v_data2 = cdt.insertVertexAndGetData(69, 69);
assert(v_data2.overlapping_vertex == v_data)
```

Inserting constraints is achieved using the `void insertConstraint(EdgeVInd edge)` method. This accepts an edge containing two vertex indices that are supposed to be connected by a constrained edge. The constrained edge cannot be changed by future additions

```
auto v_data1 = cdt.insertVertex(69, 69);
auto v_data2 = cdt.insertVertex(96, 35);

cdt.insetConstraint({4,5});
```
