# KrisLibrary

Basic C++ math, geometry, robotics, and other routines used in projects
from Kris Hauser's lab.

Authors:
- Kris Hauser (kris.hauser@duke.edu)


## Building


Requirements
- C++11: support needed in your compiler
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
- OMPL: interfaces to the Open Motion Planning Library. (Note: only works with an old verison)

To build, run:

> cmake . 
> make

or for debug information, run

> cmake . -DCMAKE_BUILD_TYPE=debug
> make

If you observe any packages not identified in the first step, run "cmake-gui ."
to potentially set the appropriate paths manually.  Let us know if you have
trouble on your system.


## Packages

  * [main folder] - basic things (error reporting, unified binary file/stream/socket I/O class, safe deletion, and timer)
  * structs - useful data structures
  * utils - assorted utilites 
  * math - a large general-purpose math library
  * math3d - 2d/3d math library
  * geometry - several computational geometry routines (e.g. convex hull)
  * optimization - convex/nonconvex optimization routines / interfaces
  * GLdraw - OpenGL wrappers and utilities
  * graph - template graph library
  * camera - projective geometry for cameras in a 3d world
  * image - basic image loading/saving
  * meshing - triangle mesh structures
  * statistics - basic statistics routines
  * robotics - robot kinematics, dynamics, and contact mechanics
  * planning - motion planning
  * spline - splines

## Known "cruft"

* The 2D libs in math3d are not quite as complete as the 3D libs
* optimization/LCP is a poor quality implementation.
* optimization/MinNormProblem does not have access to a good quadratic
  programming solver.
* The camera/ package is pretty non-conformant with the overall style.
* The GLdraw/ package is capitalized, which is non-conformant
* Image saving is very, very basic at the moment.
* Eventually everything should be placed in a unified namespace.
* Latest version of OMPL should be supported.


## Contribution guidelines

We welcome fixes and contributions to KrisLibrary.  Please follow these
guidelines when adding to the library.

- C++11 is now officially supported.  Use STL whenever possible.  It keeps things standardized and fairly portable.

- Classes should be named as precisely as possible to minimize confusion.

- Comment any confusing code.  Document using doxygen.  


### File Formatting

Encapsulate the file with the standard #ifndef CATEGORY_FILENAME_H ... #endif
where CATEGORY is the folder name, FILENAME is the file name, translated into
capitals and _ inserted between words

If a file belongs to a coherent library, the entire body (minus preprocessor
commands) should be enclosed in a namespace.  Only use `using namespace'
in .h files if it's enclosed in a namespace (e.g. Math3D inherits the
Math namespace).

### Class Formatting

3 main types of classes

* Type 1: mainly primitive objects
* Type 2: higher-level classes
* Type 3: sloppy objects that don't fall into the prev 2 types.  These should eventually be eliminated.

Classes are always capitalized (e.g. ClassName).

Methods are:
* Type 1. camel-case e.g. myFunction
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


... there is a very long and undocumented history with this package