/*---------------------------------------------------------------------------*\
 *
 *  mimmo
 *
 *  Copyright (C) 2015-2017 OPTIMAD engineering Srl
 *
 *  -------------------------------------------------------------------------
 *  License
 *  This file is part of mimmo.
 *
 *  mimmo is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU Lesser General Public License v3 (LGPL)
 *  as published by the Free Software Foundation.
 *
 *  mimmo is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mimmo. If not, see <http://www.gnu.org/licenses/>.
 *
\*---------------------------------------------------------------------------*/
# ifndef __SKDTREEUTILS_HPP__
# define __SKDTREEUTILS_HPP__

# include "bitpit_patchkernel.hpp"
# include "bitpit_surfunstructured.hpp"
# include "surface_skd_tree.hpp"
# include "volume_skd_tree.hpp"

namespace mimmo{


/*!
 * \brief Utilities employing SkdTree.
 * \ingroup core
 */
namespace skdTreeUtils{
    double distance(std::array<double,3> *P_, bitpit::PatchSkdTree *bvtree_, long &id, double &r, bool global = true);
    double signedDistance(std::array<double,3> *P_, bitpit::PatchSkdTree *bvtree_, long &id, std::array<double,3> &n, double &r, bool global = true);
    std::vector<long> selectByPatch(bitpit::PatchSkdTree *selection, bitpit::PatchSkdTree *target, double tol = 1.0e-04);
    void extractTarget(bitpit::PatchSkdTree *target, const std::vector<const bitpit::SkdNode*> & leafSelection, std::vector<long> &extracted, double tol);
    std::array<double,3> projectPoint(std::array<double,3> *P_, bitpit::PatchSkdTree *bvtree_, double r_ = 1.0e+18, bool global = true);
    long locatePointOnPatch(const std::array<double, 3> &point, bitpit::PatchSkdTree &tree);
    long closestCellToPoint(const std::array<double, 3> &point, bitpit::PatchSkdTree &tree, bool global = true);

    // Cloud version of skdtree utils methods
    //    std::vector<double> distances(std::vector<std::array<double,3>> *P_, bitpit::PatchSkdTree *bvtree_, long &id, double &r);



//#if MIMMO_ENABLE_MPI
//    inline void MPI_MAXABS(void* invec, void* inoutvec, int *len, MPI_Datatype *datatype)
//    {
//    	MPI_Aint lb, extent;
//    	MPI_Type_get_true_extent(*datatype, &lb, &extent);
//
//    	double* a = reinterpret_cast <double*> (reinterpret_cast <char*>(inoutvec) + lb);
//    	double* b = reinterpret_cast <double*> (reinterpret_cast <char*>(invec) + lb);
//
//    	for (int i = 0; i != *len; ++i)
//    	{
//    		if (std::abs(a[i]) > std::abs(b[i]))
//    			a[i] = a[i];
//    		else
//    			a[i] = b[i];
//    	}
//    }
//
//    inline void MPI_MINABS(void* invec, void* inoutvec, int *len, MPI_Datatype *datatype)
//    {
//    	MPI_Aint lb, extent;
//    	MPI_Type_get_true_extent(*datatype, &lb, &extent);
//
//    	double* a = reinterpret_cast <double*> (reinterpret_cast <char*>(inoutvec) + lb);
//    	double* b = reinterpret_cast <double*> (reinterpret_cast <char*>(invec) + lb);
//
//    	for (int i = 0; i != *len; ++i)
//    	{
//    		if (std::abs(a[i]) < std::abs(b[i]))
//    			a[i] = a[i];
//    		else
//    			a[i] = b[i];
//    	}
//    }
//#endif

}; //end namespace skdTreeUtils

} //end namespace mimmo

#endif
