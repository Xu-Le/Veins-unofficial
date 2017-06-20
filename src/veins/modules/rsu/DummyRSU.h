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

#ifndef __DUMMYRSU_H__
#define __DUMMYRSU_H__

#include <omnetpp/csimplemodule.h>

/**
 * @brief A dummy RSU used to handle with gate connection problem of the most west/east RSU.
 *
 * @author Xu Le
 * @ingroup utils
 */
class DummyRSU : public ::omnetpp::cSimpleModule
{
public:
	/** @name constructor, destructor. */
	///@{
	DummyRSU() : cSimpleModule() {}
	DummyRSU(unsigned stacksize) : cSimpleModule(stacksize) {}
	~DummyRSU() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @name gate IDs. */
	///@{
	int westIn;     ///< receive packets from west neighbor RSU.
	int westOut;    ///< send packets to west neighbor RSU.
	int eastIn;     ///< receive packets from east neighbor RSU.
	int eastOut;    ///< send packets to east neighbor RSU.
	///@}

	double positionX;
	double positionY;
	double positionZ;
};

#endif /* __DUMMYRSU_H__ */
