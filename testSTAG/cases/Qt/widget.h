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

#ifndef WIDGET_H
#define WIDGET_H

#include <vector>
#include <QString>
#include <QDebug>
#include <QWidget>
#include <QDialog>

#include "Coord.h"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
	Q_OBJECT

public:
	explicit Widget(QWidget *parent = 0);
	~Widget();

private:
	void predictVehicleMobility();

	void predictLinkBandwidth();

	void outputMobility();

private slots:
	void on_cancelButton_clicked();

	void on_nextButton_clicked();

	void on_finishButton_clicked();

private:
	Ui::Widget *ui;
	int nodeNum;
	int linkNum;
	std::vector<Coord> slotPadding;
	std::vector<std::vector<Coord> > nodePos;
	std::vector<std::vector<Coord> > nodeSpeed;
	std::vector<QString> qStrings;
};

#endif // WIDGET_H
