#include <KrisLibrary/Logger.h>
#include "quatinline.h"
#include <math/math.h>
#include <stdio.h>

namespace Math3D {

void quat_slerp(quat_t out, const quat_t a, const quat_t b, Real t)
{
	//a + b unit quaternions?
/* angle theta is defined as
cos(theta)*|a||b| = <a,b>
*/
	Real dot = quat_dot(a,b);
	if(FuzzyEquals(dot,One))		//axes are the same axis
	{
		quat_equal(out,b);
		return;
	}
	else if(FuzzyEquals(dot,-One))	//axes are opposing axis
	{
				LOG4CXX_ERROR(KrisLibrary::logger(), "Quaternions on opposing sides of unit sphere\n");
		return;
	}
	Real theta = Acos(dot);
	Real sininv = Sin(theta);
	sininv = Inv(sininv);

	quat_multiply(out, a, Sin((One-t)*theta)*sininv);
	quat_t temp;
	quat_multiply(temp, b, Sin(t*theta)*sininv);
	quat_add(out, out, temp);
}

} //namespace Math3D
