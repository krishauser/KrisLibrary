#include "frustum.h"

using namespace Camera;

void Frustum::MakeFromViewport(const Viewport& view)
{
	enum { UR=0,LR=1,UL=2,LL=3 };
	Vector3 nearPoints[4];

	Vector3 xb=view.right(),yb=view.up(),zb=view.forward();

    ///on image plane, the point that maps to v.x is fx*x+cy = v.x => x = (v.x-cy)/fx
	///the point that maps to v.x+v.w is fx*(x+w)+cy = v.x+v.w => x = (v.x+v.w-cy)/fx
	Real xmin = (view.x - view.cx)/view.fx;
	Real xmax = (view.x + view.w - view.cx)/view.fx;
	Real ymax = (view.y - view.cy)/view.fy;
	Real ymin = (view.y + view.h - view.cy)/view.fy;
	const Vector3& cp = view.pose.t;
	nearPoints[UR]=zb+xb*xmax+yb*ymax;
	nearPoints[LR]=zb+xb*xmax+yb*ymin;
	nearPoints[UL]=zb+xb*xmin+yb*ymax;
	nearPoints[LL]=zb-xb*xmin+yb*ymin;
	nearPoints[0]*=view.n; nearPoints[1]*=view.n; nearPoints[2]*=view.n; nearPoints[3]*=view.n;
	nearPoints[0] += cp; nearPoints[1] += cp; nearPoints[2] += cp; nearPoints[3] += cp;

	planes[Right].setPointBases(cp,nearPoints[LR],nearPoints[UR]);
	planes[Left].setPointBases(cp,nearPoints[UL],nearPoints[LL]);
	planes[Top].setPointBases(cp,nearPoints[UR],nearPoints[UL]);
	planes[Bottom].setPointBases(cp,nearPoints[LL],nearPoints[LR]);
	planes[Front].setPointBases(nearPoints[LR],xb,yb);
	Vector3 farPoint = cp+zb*view.f;
	planes[Back].setPointBases(farPoint,yb,xb);
}

void Frustum::MakeFromProjectionMatrix(const Matrix4& p)
{
	//if x in world coords, P*x = screen coords
	//equation for plane in world coords is n.x=o
	//find n,o, given n',o' such that n'.(Px)=o' in screen coords
	//do this with homogeneous vectors
	Matrix4 Pt;
	Pt.setTranspose(p);
	Vector4 scrnPlanes[6];
	scrnPlanes[Front].set(0,0,1,0);
	scrnPlanes[Back].set(0,0,-1,-1);
	scrnPlanes[Right].set(1,0,0,-1);
	scrnPlanes[Left].set(-1,0,0,-1);
	scrnPlanes[Top].set(0,1,0,-1);
	scrnPlanes[Bottom].set(0,-1,0,-1);
	Vector4 world;
	for(int i=0;i<6;i++) {
		Pt.mul(scrnPlanes[i],world);
		planes[i].normal.set(world.x,world.y,world.z);
		planes[i].offset=-world.w;

		Real n=planes[i].normal.norm();
		if(n!=Zero) {
			planes[i].normal/=n;
			planes[i].offset/=n;
		}
	}
}

void Frustum::MakeFromViewMatrices(const Matrix4& modelview,const Matrix4& projection)
{
	MakeFromProjectionMatrix(projection);
	Matrix4 invModelview;
	if(!invModelview.setInverse(modelview)) abort();
	Transform(invModelview);
}
