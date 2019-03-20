INCLUDE(CheckCXXSourceCompiles)
INCLUDE(CheckIncludeFileCXX)

MACRO(CHECK_FN_OR_MACRO_EXISTS _SYMBOL _HEADER _RESULT)
   SET(_INCLUDE_FILES)
   FOREACH(it ${_HEADER})
      SET(_INCLUDE_FILES "${_INCLUDE_FILES}#include <${it}>\n")
   ENDFOREACH(it)

   SET(_CHECK_PROTO_EXISTS_SOURCE_CODE "
${_INCLUDE_FILES}
int main()
{
#ifndef ${_SYMBOL}
int i = $(_SYMBOL)(0);
#endif
return 0;
}
")
   CHECK_CXX_SOURCE_COMPILES("${_CHECK_PROTO_EXISTS_SOURCE_CODE}" ${_RESULT})
ENDMACRO(CHECK_FN_OR_MACRO_EXISTS _SYMBOL _HEADER _RESULT)

# TODO: determine dynamically whether <cmath> is defined
CHECK_INCLUDE_FILE_CXX("cmath" HAS_STD_CMATH)

CHECK_FN_OR_MACRO_EXISTS(isinf  "math.h" HAS_DECL_ISINF)
CHECK_FN_OR_MACRO_EXISTS(isnan  "math.h" HAS_DECL_ISNAN)
CHECK_FN_OR_MACRO_EXISTS(isfinite  "math.h" HAS_DECL_ISFINITE)
CHECK_FN_OR_MACRO_EXISTS(finite  "math.h" HAS_DECL_FINITE)

# TODO: determine dynamically whether IEEE finite/nan comparisons are supported
SET(HAS_IEEE_COMPARISONS TRUE)
