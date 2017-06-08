//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef __GEOCASTROUTINGINTERFACE_H__
#define __GEOCASTROUTINGINTERFACE_H__

#include "veins/modules/messages/WaveShortMessage_m.h"

/**
 * @brief Interface class for geocast based routing protocol design and any concrete subclass should private inherit it.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseRoutingLayer
 */
class GeocastRoutingInterface
{
public:
    /** @name constructor, destructor. */
    ///@{
    GeocastRoutingInterface() {}
    virtual ~GeocastRoutingInterface() {}
    ///@}

protected:
    /** @name Common interfaces. */
    ///@{
    /** @brief ROI shape is rectangle. */
    virtual void disseminateROI(WaveShortMessage* wsm, Coord p1, Coord p2, Coord p3, Coord p4) = 0;
    /** @brief ROI shape is round. */
    virtual void disseminateROI(WaveShortMessage* wsm, Coord center, double radius) = 0;
    ///@}
};

#endif /* __GEOCASTROUTINGINTERFACE_H__ */
