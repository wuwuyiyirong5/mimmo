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
#ifndef __MIMMOPIERCEDVECTOR_HPP__
#define __MIMMOPIERCEDVECTOR_HPP__

#include "piercedVector.hpp"
#include "mimmoTypeDef.hpp"
#include "MimmoObject.hpp"

namespace mimmo{

/*!
 * \ingroup core
 */


/*!
 * \enum MPVLocation
 * define location of the MimmoPiercedVector data:
 *  - 0-POINT on geometry vertex
 *  - 1-CELL on geometry cells
 *  - 2-INTERFACE on geometry interfaces
 */
enum class MPVLocation{
    UNDEFINED=0,
    POINT=1,
    CELL=2,
    INTERFACE=3
};

/*!
 * \class MimmoPiercedVector
 * \brief MimmoPiercedVector is the basic data container for mimmo library
 *
 * MimmoPiercedVector is the basic container for data attached to a geometry
 * defined as a MimmoObject.
 * It is based on bitpit::PiercedVector container.
 * It supports interface methods to recover the related geometric object.
 * It supports a string name attribute to mark the field as well as a location enum to 
 * understand to which structures of geometry refers the data (UNDEFINED no-info, POINT-vertices, 
 * CELL-simplicies, INTERFACE-interfaces).
 */
template<typename value_t>
class MimmoPiercedVector: public bitpit::PiercedVector<value_t, long int> {
private:
    MimmoObject*                             m_geometry;            /**<Pointer to geometry. */
//     std::string                              m_name;                /**<Name of the field.*/
    MPVLocation                              m_loc;                 /**< MPVLocation enum */
    bitpit::Logger*                          m_log;          /**<Pointer to logger.*/
    
public:
    MimmoPiercedVector(MimmoObject* geo = NULL, MPVLocation loc = MPVLocation::UNDEFINED);
    virtual ~MimmoPiercedVector();
    //copy constructors and operators
    MimmoPiercedVector(const MimmoPiercedVector<value_t> & other);
    MimmoPiercedVector & operator=(MimmoPiercedVector<value_t> other);
    MimmoPiercedVector & operator=(bitpit::PiercedVector<value_t, long int> other);
    
    void  clear();

    // get/set methods of the class;
    MimmoObject*            getGeometry() const;
//     std::string             getName() const;
    MPVLocation             getDataLocation() const;
    std::vector<value_t>    getDataAsVector(bool ordered=false);
    
    void                   setGeometry(MimmoObject* geo);
//     void                   setName(std::string name);
    void                   setDataLocation(MPVLocation loc);
    void                   setData(std::vector<value_t> &rawdata);
    
    MPVLocation recoverGeometryReferenceLocation();
    bool checkDataSizeCoherence();
    bool checkDataIdsCoherence();

protected:
    void  swap(MimmoPiercedVector<value_t>& x) noexcept;
    
};

};

#include "MimmoPiercedVector.tpp"

#endif /* __MIMMOPIERCEDVECTOR_HPP__ */



