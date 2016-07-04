/*---------------------------------------------------------------------------*\
 *
 *  MiMMO
 *
 *  Copyright (C) 2015-2016 OPTIMAD engineering Srl
 *
 *  -------------------------------------------------------------------------
 *  License
 *  This file is part of MiMMO.
 *
 *  MiMMO is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU Lesser General Public License v3 (LGPL)
 *  as published by the Free Software Foundation.
 *
 *  MiMMO is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with MiMMO. If not, see <http://www.gnu.org/licenses/>.
 *
\*---------------------------------------------------------------------------*/
#ifndef __CONVEXHULL_HPP__
#define __CONVEXHULL_HPP__

#include "BaseManipulation.hpp"
#include <memory>
#include <unordered_set>

namespace mimmo{

/*!
 *	\date			24/mar/2016
 *	\authors		Rocco Arpa
 *	\authors		Edoardo Lombardi
 *
 *	\brief Evaluate convexHull of a 3D geometry.
 *
 *	Evaluate convex Hull via quick hull algorithm of a 3D object (Point Clouds or superficial tessellations), passed as MimmoObject. 
 *
 *	=========================================================
 * ~~~
 *	|-------------------------------------------------------------------------------------|
 *	|                    Port Input                                                       |
 *	|-------|-------------|---------------------------------------|-----------------------|
 *	|PortID | PortType    | variable/function                     | DataType		      |
 *	|-------|-------------|---------------------------------------|-----------------------|
 *	| 99    | M_GEOM      | m_geometry                            |(SCALAR, MIMMO_)       |
 *	|-------|-------------|---------------------------------------|-----------------------|
 * 
 *
 *  |---------------------------------------------------------------|
 *	|               Port Output               						|
 *	|-------|-------------|-------------------|---------------------|
 *	|PortID | PortType    | variable/function | DataType		  	|
 *	|-------|-------------|-------------------|---------------------|
 *	| 99    | M_GEOM      | getHull           |	(SCALAR, MIMMO_)	|
 *	|-------|-------------|-------------------|---------------------|
 * ~~~
 *	=========================================================
 *
 */
class OBBox: public BaseManipulation {

protected:
	std::unique_ptr<MimmoObject>	m_hull; /**< target hull */
	std::unordered_map <long, livector1D > m_visibility; /**< visibility vertices-faces map of the hull */
	std::unordered_map <long, livector1D > m_invVisibility; /**< inverse visibility faces-vertices map of the hull */
	double m_tolerance; /**! tolerance to detect concave vertices */
public:
	ConvexHull();
	virtual ~ConvexHull();

	//copy operators/constructors
	ConvexHull(const ConvexHull & other);
	ConvexHull & operator=(const ConvexHull & other);

	void buildPorts();

	//clean structure;
	void 		clearConvexHull();

	//internal methods
	double		getTolerance();
	MimmoObject * getHull();
	void		setGeometry(MimmoObject*);
	void		setTolerance(double tol);
	//plotting wrappers
	void		plot(std::string directory, std::string filename,int counter, bool binary);

	//building method
	void execute();
	
private:
	void	regularizeConvexity();
	void	startElementalTetraHedron();
	void 	createVisibleSet();
//updateVisibleSet	
};

}

#endif /* __CONVEXHULL_HPP__ */
