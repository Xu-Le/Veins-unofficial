//
// Copyright (C) 2019 Xu Le <xmutongxinXuLe@163.com>
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

#ifndef __DUMMYAPP_H__
#define __DUMMYAPP_H__

#include "veins/modules/wave/BaseWaveApplLayer.h"
#include "veins/modules/messages/DataMessage_m.h"

/**
 * @brief A dummy application -- do nothing.
 *
 * @author Xu Le
 * @ingroup waveAppLayer
 * @see BaseWaveApplLayer
 */
class DummyApp : public BaseWaveApplLayer
{
public:
	/** @brief The message kinds dummy application uses. */
	enum AppMsgKinds {
		DUMMY_APP_EVT = LAST_WAVE_APPL_MESSAGE_KIND
	};

	/** @name constructors, destructor. */
	///@{
	DummyApp() : BaseWaveApplLayer() {}
	DummyApp(unsigned stacksize) : BaseWaveApplLayer(stacksize) {}
	~DummyApp() {}
	///@}

	void initialize(int stage) override;
	void finish() override;

private:
	/** @brief handle self messages. */
	void handleSelfMsg(cMessage *msg) override;
	/** @brief handle messages from lower layer. */
	void handleLowerMsg(cMessage *msg) override;
	/** @brief handle control messages from lower layer. */
	void handleLowerControl(cMessage *msg) override;

	/** @brief call-back method of receiving data message. */
	void onData(DataMessage *dataMsg);
	/** @brief call-back method of receiving data control message. */
	void onDataLost(DataMessage *lostDataMsg);

private:
	// TODO: application specified structures
};

#endif /* __DUMMYAPP_H__ */
