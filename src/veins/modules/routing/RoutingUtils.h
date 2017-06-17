//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef __ROUTINGUTILS_H__
#define __ROUTINGUTILS_H__

#include "veins/base/utils/Coord.h"

#define Epsilon    1e-8
#define square(x)  (x) * (x)

#define ROUTING_DEBUG_LOG    1

/**
 * @brief Utility class for routing protocol design.
 *
 * @author Xu Le
 * @ingroup routingLayer
 */
class RoutingUtils
{
public:
	/** @name constructor, destructor. */
	///@{
	RoutingUtils() {}
	~RoutingUtils() {}
	///@}

	/** @name math utils. */
	///@{
	typedef Coord Vector;
	/** @brief return the length of two coords in 2D sense. */
	static double _length(Coord& point1, Coord& point2);
	/** @brief return the dot product of two vectors. */
	static double dotProduct(Coord& point1, Coord& point2);
	/** @brief return the cross product of two vectors. */
	static double crossProduct(Coord& point1, Coord& point2);
	/** @brief return the distance from a point to a line in 2D sense. */
	static double distanceToLine(Coord& P, Coord& A, Coord& B);
	/** @brief return the distance from a point to a line and the vertical point at the same time in 2D sense. */
	static double distanceToLine(Coord& P, Coord& A, Coord& B, Coord& D, double& _AD);
	///@}

	/** @brief relative direction concern with other vehicles. */
	enum DrivingDirection {
		SAME,
		OPPOSITE,
		LEFT,
		RIGHT
	};
	/** @brief obtain relative direction by absolute direction. */
	static short relativeDirection(double ownDir, double otherDir);

	/** @name routing packet's GUID utils. */
	///@{
	/** @brief return a GUID hasn't been used before. */
	static int generateGUID();
	/** @brief recycle a GUID has been used before to avoid run out of GUIDPool. */
	static void recycleGUID(int GUID);
	///@}

public:
#if RAND_MAX == 0x7fff
	static bool GUIDPool[32767]; ///< an array records whether a GUID has been used.
#else
	static bool GUIDPool[65535]; ///< an array records whether a GUID has been used.
#endif
};

#endif /* __ROUTINGUTILS_H__ */
