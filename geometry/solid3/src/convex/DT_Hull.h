/*
 * SOLID - Software Library for Interference Detection
 * 
 * Copyright (C) 2001-2003  Dtecta.  All rights reserved.
 *
 * This library may be distributed under the terms of the Q Public License
 * (QPL) as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE.QPL included in the packaging of this file.
 *
 * This library may be distributed and/or modified under the terms of the
 * GNU General Public License (GPL) version 2 as published by the Free Software
 * Foundation and appearing in the file LICENSE.GPL included in the
 * packaging of this file.
 *
 * This library is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Commercial use or any other use of this library not covered by either 
 * the QPL or the GPL requires an additional license from Dtecta. 
 * Please contact info@dtecta.com for enquiries about the terms of commercial
 * use of this library.
 */

#ifndef DT_HULL_H
#define DT_HULL_H
#include <iostream>

#include "DT_Convex.h"

class DT_Hull : public DT_Convex {
public:
	DT_Hull(const DT_Convex& lchild, const DT_Convex& rchild) :
		m_lchild(lchild), 
		m_rchild(rchild) 
	{}

	virtual MT_Scalar supportH(const MT_Vector3& v) const 
	{
		return GEN_max(m_lchild.supportH(v), m_rchild.supportH(v));
	}

	virtual MT_Point3 support(const MT_Vector3& v) const 
	{
		MT_Point3 lpnt = m_lchild.support(v);
		MT_Point3 rpnt = m_rchild.support(v);
		return v.dot(lpnt) > v.dot(rpnt) ? lpnt : rpnt;
	}

private:
	const DT_Convex& m_lchild;
	const DT_Convex& m_rchild;
};

// construct a convex hull from one object and rigidtransform it
class DT_HullTran : public DT_Convex {
public:
	DT_HullTran(const DT_Convex& lchild) :
		m_child(lchild)
	{
		m_xform.setIdentity();
	}

	virtual MT_Scalar supportH(const MT_Vector3& v) const 
	{
		MT_Scalar v1 = m_child.supportH(v);
		MT_Scalar v2 = m_child.supportH(v * m_xform.getBasis()) + v.dot(m_xform.getOrigin());
		//std::cout << "Tran supportH v = " << v[0] << " " << v[1] << " " << v[2] << " ";
		//std::cout << "v1 = " << v1 << " v2 = " << v2 << std::endl;
		return GEN_max(v1, v2);
	}

	virtual MT_Point3 support(const MT_Vector3& v) const 
	{
		MT_Point3 lpnt = m_child.support(v);
		MT_Point3 rpnt = m_xform(m_child.support(v * m_xform.getBasis()));
		//std::cout << "Tran support v = " << v[0] << " " << v[1] << " " << v[2] << std::endl;
		//std::cout << "lpnt = " << lpnt[0] << " " << lpnt[1] << " " << lpnt[2] << std::endl;
		//std::cout << "rpnt = " << rpnt[0] << " " << rpnt[1] << " " << rpnt[2] << std::endl;
		return v.dot(lpnt) > v.dot(rpnt) ? lpnt : rpnt;
	}

	void _update_transform(const double *pointer) {
		//
		//std::cout << "update transform here\n";
		m_xform.setValue(pointer);
	}

private:
	const DT_Convex& m_child;
	MT_Transform m_xform;
};

// construct a convex hull from two shapes where the second shape freely transforms
class DT_HullFree : public DT_Convex {
public:
	DT_HullFree(const DT_Convex& lchild, const DT_Convex &rchild) :
		m_lchild(lchild), 
		m_rchild(rchild) 
	{
		m_xform.setIdentity();
	}

	virtual MT_Scalar supportH(const MT_Vector3& v) const 
	{
		MT_Scalar v1 = m_lchild.supportH(v);
		MT_Scalar v2 = m_rchild.supportH(v * m_xform.getBasis()) + v.dot(m_xform.getOrigin());
		return GEN_max(v1, v2);
	}

	virtual MT_Point3 support(const MT_Vector3& v) const 
	{
		MT_Point3 lpnt = m_lchild.support(v);
		MT_Point3 rpnt = m_xform(m_rchild.support(v * m_xform.getBasis()));
		return v.dot(lpnt) > v.dot(rpnt) ? lpnt : rpnt;
	}

	void _update_transform(const double *pointer) {
		m_xform.setValue(pointer);
	}


private:
	const DT_Convex& m_lchild;
	const DT_Convex& m_rchild;
	MT_Transform m_xform;
};

#endif
