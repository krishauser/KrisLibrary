# KrisLibrary

Basic C++ math, geometry, robotics, I/O, and other routines used in projects
from Kris Hauser's lab. 

Authors:
- Kris Hauser (kris.hauser@duke.edu)


## Building

Requirements
- C++11: must be supported in your compiler
- CMake: needed to configure and build KrisLibrary
- OpenGL

Optional
- GLUT / GLUI: needed to open KrisLibrary GLdraw/GLUT* or GLdraw/GLUI* visualization windows
- GLPK: needed for optimization and contact mechanics.  
- Tinyxml: needed for Xml resource loading.  
- GSL: needed for the special functions in math/misc.h.
- Assimp: for extra triangle mesh file format loading.
- BLAS/LAPACK (disabled by default): needed only for the experimental
  BLAS and LAPACK interfaces through CLAPACK.  
- Log4CXX: for custom logging functionality
- FreeImage: needed for extra image file format loading (JPG,PNG,TIF,GIF, etc)
- OMPL: interfaces to the Open Motion Planning Library. (Note: only works with an older verison)

To build, run:

> cmake . 
> make

or for debug information, run

> cmake . -DCMAKE_BUILD_TYPE=debug
> make

If you observe that any packages on your system are not identified in the first step, run

> cmake-gui .

to set the appropriate paths manually.  These are variables in the form of `X_ROOT`.  You may also
disable certain packages by turning off the `USE_X` variables.  Please submit an issue on Github
if you have trouble building on your system.


## Packages

  * [main folder] - standard useful items (error reporting, unified binary file/stream/socket I/O class, safe deletion, and timer)
  * structs - useful data structures
  * utils - assorted utilites 
  * math - a large general-purpose math library
  * math3d - 2d/3d math library
  * geometry - several computational geometry routines (e.g. collision detection, convex hull)
  * optimization - convex/nonconvex optimization routines / interfaces
  * GLdraw - OpenGL wrappers and utilities
  * graph - template graph library
  * camera - projective geometry for cameras in a 3d world
  * image - basic image loading/saving
  * meshing - triangle mesh, volume mesh classes and routines
  * statistics - basic statistics routines
  * robotics - robot kinematics, dynamics, and contact mechanics
  * planning - motion planning
  * spline - splines

## Features

- Unified I/O to files, sockets, and memory buffers using the File object.
- Undirected graphs, directed graphs, and trees.  Dijkstra's algorithm and A* are supported.  Vertices and edges can store arbitrary datatypes.
- 2D, 3D, N-d, and sparse arrays.
- An "any" datatype that can contain objects of arbitrary type (based on boost::any).
- JSON loading and saving via AnyCollection.
- 2D and 3D math library, including many types of rotation representations (rotation matrix, quaternions, axis-angle).
- Wide variety of geometries supported: triangle meshes, point clouds, volume grids, and geometric primitives.  Supports loading, saving, collision detection, and conversion between types.
- 

## Known "cruft"

* The 2D libs in math3d are not quite as complete as the 3D libs
* optimization/LCP is a poor quality implementation.
* optimization/MinNormProblem does not have access to a good quadratic
  programming solver.
* The GLdraw/ package is capitalized, which is non-conformant
* Image saving is very basic at the moment.
* OFF file loading only supports triangle meshes
* Eventually everything should be placed in a unified namespace.
* The utils/stl_tr1.h file was a workaround for the unordered_set / undordered_map containers that was useful
  while STL was in flux around the time of adoption of C++11.  Now this looks standardized... Can it be safely replaced?
* Latest version of OMPL should be supported.
* Logger calls are still calling LOG4CXX, even if LOG4CXX is not used.  Maybe this should be renamed to KL_LOG or something similar.

## Contribution guidelines

We welcome fixes and contributions to KrisLibrary.  Please follow these
guidelines when adding to the library.

- C++11 is now officially supported.  Use STL whenever possible.  It keeps things standardized and fairly portable.

- Classes should be named as precisely as possible to minimize confusion.

- Comment any confusing code.  Document using doxygen.  


### File Formatting

Encapsulate the file with the standard #ifndef CATEGORY_FILENAME_H ... #endif
where CATEGORY is the folder name, FILENAME is the file name, translated into
capitals and _ inserted between words.

If a file belongs to a coherent library, the entire body (minus preprocessor
commands) should be enclosed in a namespace.  Only use `using namespace'
in .h files if it's enclosed in a namespace (e.g. Math3D inherits the
Math namespace).

### Class Formatting

3 main types of classes

* Type 1: mainly primitive objects
* Type 2: higher-level classes
* Type 3: sloppy objects that don't fall into the prev 2 types.  These should eventually be eliminated.

Classes are always capitalized camel-case (e.g. ClassName).

Methods are:
* Type 1. camel-case beginning with a lowercase letter e.g. myFunction
* Type 2. Capitalized

Member variables are always lowercase.

### Global Formatting:

Non-inlined functions are always capitalized.  Inlined functions may be
lowercase (usually reserved for very primitive operations, like dot products).
Macros are either all caps or capitalized.
Variables are lowercase.


### Serialization

Whenever possible, objects should be able to be loaded from/saved to disk
using the << and >> operators of the standard iostream objects.  For
binary I/O, you can implement the WriteFile(object,File&) and
ReadFile(object,File&) methods.


## History

* 12/14/2018 (Latest version): upgraded to take out Boost dependencies, officially using C++11, optional LOG4CXX support, bug fixes.

... prior to that, there is a long and undocumented history stretching back over a decade!

## Questions for future development

- Most of the vector / matrix operations could be replaced with another library e.g., Eigen.  But this means replacing a lot of 
  legacy code in the optimization, robotics, and statistics packages. Is this worth doing?

- When this was first developed in the 2000's, there were few cross-platform C++ libraries for doing OS, I/O, and numerical
  operations, so the routines here filled a need.  This has changed significantly with libraries like POCO, Boost, and language 
  upgrades to C++11 and beyond.

- Many functions don't take advantage of the move semantics introduced in C++11.  So almost all method signatures are like
  `void get(Output& result)` rather than `Output get()`.  Should this be changed to make it the code more modern?
