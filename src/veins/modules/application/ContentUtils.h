//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __CONTENTUTILS_H__
#define __CONTENTUTILS_H__

#include <algorithm>
#include "veins/base/utils/Coord.h"
#include "veins/base/utils/SimpleAddress.h"

typedef std::list<LAddress::L3Type> NeighborItems;

/** Segment structure of data. */
class Segment
{
public:
	Segment();
	~Segment();

	Segment& operator=(const Segment& rhs);

	void assign(const Segment *rhs);
	void print();

	int begin;
	int end;
	Segment *next;
};

/**
 * @brief Utility class for content downloading application.
 *
 * @author Xu Le
 * @ingroup applLayer
 */
class ContentUtils
{
public:
	/** @name constructor, destructor. */
	///@{
	ContentUtils() {}
	~ContentUtils() {}
	///@}

	/** @brief transform total application layer bytes to total link layer bytes.
	 *
	 * @param applBytes total application layer bytes;
	 * @param headerLen header length of single packet measured in bytes;
	 * @param dataLen max data length of single packet measured in bytes;
	 * @return total application layer bytes.
	 */
	static int calcLinkBytes(int applBytes, int headerLen, int dataLen);

	/** @brief apply binary search to a sorted std::vector. */
	template <typename T>
	static bool vectorSearch(std::vector<T>& vec, const T key)
	{
		int low = 0, high = static_cast<int>(vec.size())-1, mid = 0;
		while (low <= high)
		{
			mid = low + ((high - low) >> 1);
			if (key == vec[mid])
				return true;
			else if (key < vec[mid])
				high = mid - 1;
			else
				low = mid + 1;
		}
		return false;
	}
	/** @brief remove an element in std::vector<T>. */
	template <typename T>
	static void vectorRemove(std::vector<T>& vec, const T key)
	{
		vec.erase(std::remove(vec.begin(), vec.end(), key), vec.end());
	}

public:
	static int rateTable[50]; ///< measured in kbps.
};

#endif /* __CONTENTUTILS_H__ */
