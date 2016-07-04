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
 \ *---------------------------------------------------------------------------*/

#include "ConvexHull.hpp"

#include <chrono>

using namespace std::chrono;

using namespace std;
using namespace mimmo;

/*
 *	\date			1/jul/2016
 *	\authors		Rocco Arpa
 *	\authors		Edoardo Lombardi
 *
 *	 Evaluate convexHull of a 3D geometry.
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

/*! Basic Constructor. Doing nothing.*/
ConvexHull::ConvexHull(){
	m_name = "MiMMO.ConvexHull";
	m_tolerance = 1.E-6;
};

/*! Destructor */
ConvexHull::~ConvexHull(){};

/*! Copy Constructor
 *\param[in] other ConvexHull where copy from
 */
ConvexHull::ConvexHull(const ConvexHull & other){
	*this = other;
};

/*! Copy Operator(does not copy anything else then input parameters)
 * \param[in] other ConvexHull where copy from
 */
ConvexHull & ConvexHull::operator=(const ConvexHull & other){

	*(static_cast<BaseManipulation *>(this))  = *(static_cast<const BaseManipulation *>(&other));
	m_tolerance = other.m_tolerance;
	return(*this);
};

/*! It builds the input/output ports of the object
 */
void ConvexHull::buildPorts(){

	bool built = true;
//creating input ports	
	built = (built && createPortIn<MimmoObject*, ConvexHull>(&m_geometry, M_GEOM, mimmo::pin::containerTAG::SCALAR, mimmo::pin::dataTAG::MIMMO_));
// creating output ports
	built = (built && createPortOut<MimmoObject*, ConvexHull>(this, &mimmo::ConvexHull::getHull, M_GEOM, mimmo::pin::containerTAG::SCALAR, mimmo::pin::dataTAG::MIMMO_));
	
	m_arePortsBuilt = built;
};

/*!Clean all stuffs in your class */
void ConvexHull::clearConvexHull(){
	clear(); //base manipulation stuff clear
	m_visibility.clear();
	m_invVisibility.clear();
	m_tolerance = 1.0E-6;
	m_hull.reset(nullptr);
};

/*! 
 * Return tolerance for convexity detection in convex hull
 */
double	ConvexHull::getTolerance(){
	return(m_tolerance);
}

/*! 
 * Return pointer to a MimmoObject structure containing the Hull
 */
MimmoObject * 	ConvexHull::getHull(){
	return(m_hull.get());
}

/*!
 * Set your target geometry. Reimplemented from BaseManipulation::setGeometry().
 */
void	ConvexHull::setGeometry(MimmoObject * geo){
	BaseManipulation::setGeometry(geo);
};

/*! 
 * Set tolerance for convexity detection in convex hull
 */
void	ConvexHull::setTolerance(double tol){
	m_tolerance = tol;
}


/*! Plot the OBB as a structured grid to *vtu file.
 * \param[in] directory output directory
 * \param[in] filename  output filename w/out tag
 * \param[in] counter   integer identifier of the file
 * \param[in] binary    boolean flag for 0-"ascii" or 1-"appended" writing
 */
void		ConvexHull::plot(std::string directory, std::string filename,int counter, bool binary){
	

	dvecarr3E activeP = m_hull->getVertexCoords();
	ivector2D activeConn = m_hull->getCompactConnectivity();

	bitpit::VTKFormat codex = bitpit::VTKFormat::ASCII;
	if(binary){codex=bitpit::VTKFormat::APPENDED;}
	bitpit::VTKElementType elDM = bitpit::VTKElementType::TRIANGLE;
	bitpit::VTKUnstructuredGrid vtk(directory, filename, elDM);
	vtk.setGeomData( bitpit::VTKUnstructuredField::POINTS, activeP) ;
	vtk.setGeomData( bitpit::VTKUnstructuredField::CONNECTIVITY, activeConn) ;
	vtk.setDimensions(activeConn.size(), activeP.size());
	vtk.setCodex(codex);
	if(counter>=0){vtk.setCounter(counter);}
	
	vtk.write();
};


/*!Execute your object, calculate the ConvexHull of your geometry. Implementation of pure virtual BaseManipulation::execute
 */
void 		ConvexHull::execute(){
	
	
};

/*! 
 * Calculates and returns the eigenVectors and eigenvalues of a 3x3 matrix.
 * \param[in] matrix	target matrix
 * \param[out]	eigenvalues eigenvalues of the matrix
 * \return	matrix of eigenvectors by column
 */
void 		ConvexHull::regularizeConvexity{
}

/*! 
 * Calculates and returns the eigenVectors and eigenvalues of a 3x3 matrix.
 * \param[in] matrix	target matrix
 * \param[out]	eigenvalues eigenvalues of the matrix
 * \return	matrix of eigenvectors by column
 */
void 		ConvexHull::startElementalTetraHedron{
}

/*! 
 * Calculates and returns the eigenVectors and eigenvalues of a 3x3 matrix.
 * \param[in] matrix	target matrix
 * \param[out]	eigenvalues eigenvalues of the matrix
 * \return	matrix of eigenvectors by column
 */
void 		ConvexHull::createVisibleSet{
}
