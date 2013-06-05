KrisLibrary
===========

Basic math, geometry, and robotics, and other routines used in projects
from Kris Hauser's lab.

Authors:
- Kris Hauser (hauserk@indiana.edu)


********************************************************************
* Building
********************************************************************
Copy Makefile.config.tmpl to Makefile.config.  Pre-configured config
files for your system may be available in Makefile.config.[system].

Edit paths and optional packages in Makefile.config.
- GLUT / GLUI: almost always needed.  Configure the GLUT and GLUI paths
  to point to the right places on your system. To disable, set HAVE_GLUT
  and/or HAVE_GLUI to 0. 
- GLPK: needed for optimization and contact mechanics.  To disable, set
  HAVE_GLPK to 0.
- Tinyxml: needed for Xml resource loading.  To disable, set HAVE_TINYXML to 0.
- GSL (disabled by default): needed only if you plan to use the special
  functions in math/misc.h.
- Assimp (disabled by default): needed only for "exotic" triangle mesh
  loading.  To enable, set HAVE_ASSIMP to 1 and set the appropriate ASSIMP 
  path.
- BLAS/LAPACK (disabled by default): needed only for the experimental
  BLAS and LAPACK interfaces through CLAPACK.  To enable, set HAVE_BLAS and
  HAVE_LAPACK to 1, and set the appropriate CLAPACK_DIR path.

Build the library using 'make'.  Documentation may also be built using
'make docs'.



********************************************************************
* Packages
********************************************************************
  . structs - useful data structures
  . utils - assorted utilites 
  . math - a large general-purpose math library
  . math3d - 2d/3d math library
  . geometry - several computational geometry routines (e.g. convex hull)
  . optimization - convex/nonconvex optimization routines / interfaces
  . GLdraw - OpenGL wrappers and utilities
  . graph - template graph library
  . camera - projective geometry for cameras in a 3d world
  . image - basic image loading/saving
  . meshing - triangle mesh structures
  . statistics - basic statistics routines
  . robotics - robot kinematics, dynamics, and contact mechanics
  . planning - motion planning
  . splines - splines

********************************************************************
* Known "cruft"
********************************************************************
- myfile should be renamed.
- the 2D libs in math3d are not quite as complete as the 3D libs
- GLdraw should be renamed.
- optimization/LCP is a poor quality implementation.
- optimization/MinNormProblem does not have access to a good quadratic
  programming solver.
- camera/clip is named the same as math3d/clip, and is a less powerful
  reimplementation of math3d/Polyhedron3D.

********************************************************************
* Contribution guidelines
********************************************************************

We welcome fixes and contributions to KrisLibrary.  Please follow these
guidelines when adding to the library.

Classes should be named as precisely as possible to minimize confusion.

Comment any confusing code.  Document using doxygen.  Write notes like "TODO"
or "HACK" for partially complete code.

Use STL whenever possible.  It keeps things standardized and fairly portable.


Structures vs. Algorithms

Data structure classes should be very reusable.  Basic primitive methods should be included, but the class should not contain complex algorithms.  For example, a LinearProgram does not contain any information about how to solve it.

Algorithms should be implemented as either functions or external operator classes (latter preferred for complex algorithms -- this allows greater flexibility in defining algorithm parameters, options, control flow, etc).
  There are 3 phases in an operator class A: initialization, calculation, and result output.
  Initialization could be in a constructor "A a(input)" or an initializer "a.Initialize(input)" or simultaneously with calculation "a.Calculate(input)".  This could also includes setting options, perhaps by directly setting member variables.
  The output could either be output as an argument to the calculation method "a.Calculate(input,result)" or a member variable in the operator "a.result".




****************************************************************************
****************************************************************************

File Formatting:

Encapsulate the file with the standard #ifndef CATEGORY_FILENAME_H ... #endif
where CATEGORY is the folder name, FILENAME is the file name, translated into
capitals and _ inserted between words

If a file belongs to a coherent library, the entire body (minus preprocessor
commands) should be enclosed in a namespace.  Only use `using namespace'
in .h files if it's enclosed in a namespace (e.g. Math3D inherits the
Math namespace).

****************************************************************************

Class Formatting:

3 main types of classes
Type 1 is mainly primitive objects
Type 2 is higher-level classes
Type 3 is sloppy objects that don't fall into the prev 2 types.  These should eventually be eliminated.

classes are always capitalized (e.g. ClassName).
methods are:
  Type 1. start with lowercase, subsequent words capitalized e.g. myFunction
  Type 2. Capitalized
member variables are always lowercase.

****************************************************************************

Global Formatting:

Non-inlined functions always capitalized.  Inlined functions may be
lowercase (usually reserved for very primitive operations, like dot products).
Macros are either all caps or capitalized.
Variables are lowercase.

****************************************************************************

Serialization

Whenever possible, objects should be able to be loaded from/saved to disk
using the << and >> operators of the standard iostream objects.  For
binary I/O, classes should implement Load(File&) and Save(File&) methods.

****************************************************************************


