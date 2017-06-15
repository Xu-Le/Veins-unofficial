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

#include <omnetpp/clog.h>

using omnetpp::getThisPtr;

/** Segment structure of data. */
struct Segment
{
    Segment();
    ~Segment();

    void assign(const Segment *rhs);
    void print();

    int begin;
    int end;
    struct Segment *next;
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

public:
    static int rateTable[25]; ///< measured in kbps.
};

#endif /* __CONTENTUTILS_H__ */
