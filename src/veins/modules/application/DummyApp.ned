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

package org.car2x.veins.modules.application;

import org.car2x.veins.modules.wave.BaseWaveApplLayer;

simple DummyApp extends BaseWaveApplLayer
{
	@class(DummyApp);
	@display("i=block/app");
	int dataLengthBits = default(18432bit) @unit(bit); // the length of a data packet 8 * 2304
	int dataPriority = default(1); //the access category for data packets
}
