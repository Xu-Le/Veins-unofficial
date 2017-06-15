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

#ifndef __BROADCASTROUTINGINTERFACE_H__
#define __BROADCASTROUTINGINTERFACE_H__

#include "veins/modules/messages/WarningMessage_m.h"

/**
 * @brief Interface class for broadcast based routing protocol design and any concrete subclass should private inherit it.
 *
 * @author Xu Le
 * @ingroup routingLayer
 * @see BaseRoutingLayer
 */
class BroadcastRoutingInterface
{
public:
	/** @name constructor, destructor. */
	///@{
	BroadcastRoutingInterface() {}
	virtual ~BroadcastRoutingInterface() {}
	///@}

protected:
	/** @name Common interfaces. */
	///@{
	/**
	 * @brief Execute broadcast storm mitigation techniques to alleviate link load, message delay and packet loss.
	 *
	 * reference: BROADCAST STORM MITIGATION TECHNIQUES IN VEHICULAR AD HOC NETWORKS, IEEE Wireless Communications, December 2007
	 */
	virtual void broadcastSuppression(WarningMessage* wsm) = 0;
	/** @brief Rebroadcast the packet received from neighbors. */
	virtual void rebroadcast(WarningMessage* wsm) = 0;
	///@}
};

#endif /* __BROADCASTROUTINGINTERFACE_H__ */
