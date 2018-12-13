/**********************************************************************
 * Primitive Format:
 * class ClassName
 * {
 *   ClassName()		constructors (WARNING default should not initialize anything -- not zero!)
 *   operator*=()		inplace operators
 *   operator Other()	cast operators
 *   op()				efficient operator methods in the form this=arg1 op arg2 ...
 *   setX()				initializers
 *   getX()				extractors
 *   inplaceX()			inplace modifiers
 *   load/save()		binary file io
 *   otherMethods()		other methods
 *
 *   attributes
 * }
 * external functions
 **********************************************************************/
