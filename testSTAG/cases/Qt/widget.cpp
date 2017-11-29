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

#include "widget.h"
#include "mydialog.h"
#include "ui_widget.h"
#include <fstream>
#include <QMessageBox>

const Coord Coord::ZERO = Coord(0.0, 0.0, 0.0);

int downloaderNum = 0;
int slotNum = 0;
int caseIndex = 0;

static int rateTable[50] = { 5600, 5600, 5500, 5500, 5500, 5500, 5400, 5400, 5300, 5300,
5300, 5300, 5200, 5200, 5200, 5200, 5200, 5200, 5200, 5200,
5100, 5100, 5100, 5100, 5000, 4900, 4700, 4400, 4100, 3750,
3400, 3000, 2600, 2100, 1600, 1150, 700, 550, 400, 400,
350, 350, 300, 300, 300, 250, 250, 250, 200, 200 };

class LinkTuple
{
public:
	LinkTuple(int _src, int _dst, int _slot_num) : src(_src), dst(_dst), slotNum(_slot_num) { bandwidth = new int[slotNum]; }
	~LinkTuple() { delete []bandwidth; }

	LinkTuple(const LinkTuple& rhs)
	{
		src = rhs.src;
		dst = rhs.dst;
		slotNum = rhs.slotNum;
		bandwidth = new int[slotNum];
		memcpy(bandwidth, rhs.bandwidth, slotNum*sizeof(int));
	}

	int src;
	int dst;
	int slotNum;
	int *bandwidth;

private:
	LinkTuple& operator=(const LinkTuple&);
};


Widget::Widget(QWidget *parent) : QWidget(parent), ui(new Ui::Widget)
{
	ui->setupUi(this);
	QString firstLine("RSU      0    0    0    0");
	ui->infoBrowser->setText(firstLine);
	qStrings.push_back(firstLine);
	nodeNum = 1;

	slotPadding.push_back(Coord::ZERO);
	nodeSpeed.push_back(slotPadding);
	nodePos.push_back(slotPadding);
}

Widget::~Widget()
{
	delete ui;
}

void Widget::predictVehicleMobility()
{
	for (int j = 1; j <= slotNum; ++j)
		for (int i = 1; i < nodeNum; ++i)
			nodePos[i][j] = nodePos[i][j-1] + nodeSpeed[i][0];
}

void Widget::predictLinkBandwidth()
{
	linkNum = nodeNum * (nodeNum - 1) / 2;

	for (int i = 0; i < nodeNum; ++i)
	{
		nodePos[i].resize(slotNum+1, Coord::ZERO);
		nodeSpeed[i].resize(slotNum+1, Coord::ZERO);
	}

	predictVehicleMobility();

	std::vector<int> distPadding(nodeNum);
	std::vector<std::vector<int> > nodeDist;
	for (int nodeId = 0; nodeId < nodeNum; ++nodeId)
		nodeDist.push_back(distPadding);

	int srcNode = 0, dstNode = 0;
	std::list<LinkTuple> linkTuples;
	for (srcNode = 0; srcNode < nodeNum-1; ++srcNode)
	{
		for (dstNode = srcNode+1; dstNode < nodeNum; ++dstNode)
		{
			linkTuples.push_back(LinkTuple(srcNode, dstNode, slotNum));
			memset(linkTuples.back().bandwidth, 0, slotNum*sizeof(int));
			nodeDist[srcNode][dstNode] = Coord::length(nodePos[srcNode][0], nodePos[dstNode][0]);
		}
	}

	std::list<LinkTuple>::iterator itLT = linkTuples.begin();
	for (int j = 0; j < slotNum; ++j)
	{
		itLT = linkTuples.begin();
		for (srcNode = 0; srcNode < nodeNum-1; ++srcNode)
		{
			for (dstNode = srcNode+1; dstNode < nodeNum; ++dstNode)
			{
				int curDist = nodeDist[srcNode][dstNode];
				int nextDist = Coord::length(nodePos[srcNode][j+1], nodePos[dstNode][j+1]);
				nodeDist[srcNode][dstNode] = nextDist;
				if (curDist < 250 || nextDist < 250)
				{
					int minIdx = curDist/5, maxIdx = nextDist/5;
					if (minIdx > maxIdx)
						std::swap(minIdx, maxIdx);
					int maxIdx_ = std::min(49, maxIdx);
					for (int i = minIdx; i <= maxIdx_; ++i)
						itLT->bandwidth[j] += 128*rateTable[i];
					itLT->bandwidth[j] /= maxIdx - minIdx + 1;
				}
				else
					itLT->bandwidth[j] = 0;
				++itLT;
			}
		}
	}

	char caseFile[] = "case00.txt";
	if (caseIndex >= 10)
		caseFile[4] += caseIndex/10;
	caseFile[5] += caseIndex%10;
	std::ofstream fout(caseFile, std::ios_base::out | std::ios_base::trunc);
	if (!fout.is_open())
	{
		qDebug() << "cannot open case file!" << endl;
		exit(EXIT_FAILURE);
	}

	fout << nodeNum << " " << linkNum << " " << downloaderNum << " " << slotNum << "\n\n";
	for (itLT = linkTuples.begin(); itLT != linkTuples.end(); ++itLT)
	{
		fout << itLT->src << " " << itLT->dst;
		for (int j = 0; j < slotNum; ++j)
			fout << " " << itLT->bandwidth[j];
		fout << "\n";
	}
	fout << std::endl;

	for (int nodeId = 0; nodeId < nodeNum; ++nodeId)
	{
		fout << nodeId;
		for (int d = 0; d < downloaderNum; ++d)
		{
			if (nodeId > 0)
				fout << " 1 0";
			else // RSU self
				fout << " 1 99999999";
		}
		fout << "\n";
	}
	fout << std::endl;

	fout.close();
}

void Widget::on_cancelButton_clicked()
{
	if (nodeNum > 1)
	{
		--nodeNum;
		qStrings.pop_back();
		nodePos.pop_back();
		nodeSpeed.pop_back();
		ui->infoBrowser->clear();
		for (int i = 0; i < nodeNum; ++i)
			ui->infoBrowser->append(qStrings[i]);
	}
}

void Widget::on_nextButton_clicked()
{
	double pos_x = ui->posXSpinBox->value();
	double pos_y = ui->posYSpinBox->value();
	double speed_x = ui->speedXSpinBox->value();
	double speed_y = ui->speedYSpinBox->value();
	nodePos.push_back(slotPadding);
	nodeSpeed.push_back(slotPadding);
	nodePos[nodeNum][0].x = pos_x;
	nodePos[nodeNum][0].y = pos_y;
	nodeSpeed[nodeNum][0].x = speed_x;
	nodeSpeed[nodeNum][0].y = speed_y;

	QString vehIDString, posXString, posYString, speedXString, speedYString;
	vehIDString.setNum(nodeNum, 10);
	posXString.setNum(pos_x, 'g', 3);
	posYString.setNum(pos_y, 'g', 3);
	speedXString.setNum(speed_x, 'g', 3);
	speedYString.setNum(speed_y, 'g', 3);
	QString curLine("veh");
	curLine.append(vehIDString);
	if (nodeNum >= 10)
		curLine.append("  ");
	else
		curLine.append("    ");
	curLine.append(posXString);
	curLine.append("    ");
	curLine.append(posYString);
	curLine.append("    ");
	curLine.append(speedXString);
	curLine.append("    ");
	curLine.append(speedYString);
	ui->infoBrowser->append(curLine);
	qStrings.push_back(curLine);
	++nodeNum;
}

void Widget::outputMobility()
{
	char caseFile[] = "traffic00.txt";
	if (caseIndex >= 10)
		caseFile[7] += caseIndex/10;
	caseFile[8] += caseIndex%10;
	std::ofstream fout(caseFile, std::ios_base::out | std::ios_base::trunc);
	if (!fout.is_open())
	{
		qDebug() << "cannot open traffic file!" << endl;
		exit(EXIT_FAILURE);
	}

	for (int i = 0; i < nodeNum; ++i)
		fout << i << ' ' << nodePos[i][0].x << ' ' << nodePos[i][0].y << ' ' << nodeSpeed[i][0].x << ' ' << nodeSpeed[i][0].y << "\n";

	fout.close();
}

void Widget::on_finishButton_clicked()
{
	MyDialog myDialog;
	myDialog.exec();
	predictLinkBandwidth();
	outputMobility();
	QMessageBox::information(this, "information", "STAG generated!", QMessageBox::Yes);
	close();
}

