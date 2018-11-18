//
// Copyright (C) 2018-2019 Xu Le <xmutongxinXuLe@163.com>
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

#include "mainwindow.h"
#include "ui_mainwindow.h"

double gX = 0.0;
double gY = 0.0;
int numVehicle = 0;
Point *vehicleTable = NULL;
double *velocityTable = NULL;
double *directionTable = NULL;

const char *MainWindow::cstringDigits[30] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29" };

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow),
	solution(),
	running(1),
	timesFont("Times", 10),
	solidPen(Qt::SolidLine),
	dashPen(Qt::DashLine),
	rsuIndexText(NULL),
	rsuInnerCircle(NULL),
	rsuOutterCircle(NULL)
{
	ui->setupUi(this);
	setGeometry(400, 100, 600, 610); // 1120
	solidPen.setColor(Qt::darkGreen);
	dashPen.setColor(Qt::darkGreen);

	QMenu *editMenu = ui->menuBar->addMenu(tr("edit"));
	QAction *pauseAction = editMenu->addAction(tr("pause"), this, SLOT(pauseEvent()));
	QAction *continueAction = editMenu->addAction(tr("continue"), this, SLOT(continueEvent()));
	ui->toolBar->addAction(pauseAction);
	ui->toolBar->addAction(continueAction);

	int uavNum = parseInput("case.txt");
	configEnv();

	elapsed = solution.initialize(uavNum);
	uavIndices.reserve(uavNum);
	uavInnerCircles.reserve(uavNum);
	uavOutterCircles.reserve(uavNum);

	initialPlot();

	// setup a timer that repeatedly calls MainWindow::scheduleEvent()
	connect(&eventTimer, SIGNAL(timeout()), this, SLOT(scheduleEvent()));
	eventTimer.start(100);
}

MainWindow::~MainWindow()
{
	delete []vehicleTable;
	delete []velocityTable;
	delete []directionTable;
	delete ui;
}

void MainWindow::initialPlot()
{
	QPen pen(Qt::SolidLine);
	pen.setColor(Qt::blue);

	QCPGraph *graph0 = ui->customPlot->addGraph();
	graph0->setPen(pen);
	graph0->setLineStyle(QCPGraph::lsNone);
	graph0->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 2));

	QVector<double> xCoord(numVehicle), yCoord(numVehicle);
	for (int i = 0; i < numVehicle; ++i)
	{
		xCoord[i] = vehicleTable[i].x;
		yCoord[i] = vehicleTable[i].y;
	}
	// configure right and top axis to show ticks but no labels:
	ui->customPlot->xAxis2->setVisible(true);
	ui->customPlot->xAxis2->setTickLabels(false);
	ui->customPlot->yAxis2->setVisible(true);
	ui->customPlot->yAxis2->setTickLabels(false);
	// make left and bottom axes always transfer their ranges to right and top axes:
	connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
	connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));
	// pass data points to graphs:
	graph0->setData(xCoord, yCoord);
	// give the axes some labels:
	// ui->customPlot->xAxis->setLabel("x");
	// ui->customPlot->yAxis->setLabel("y");
	// set axes ranges, so we see all data:
	ui->customPlot->xAxis->setRange(0, gX);
	ui->customPlot->yAxis->setRange(0, gY);
	// let the ranges scale themselves so graph 0 fits perfectly in the visible area:
	// graph0->rescaleAxes();
	// Allow user to drag axis ranges with mouse, zoom with mouse wheel:
	ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

	RSU *rsu = &solution.rsu;
	rsuIndexText = new QCPItemText(ui->customPlot);
	rsuIndexText->setPositionAlignment(Qt::AlignmentFlag::AlignCenter);
	rsuIndexText->position->setCoords(rsu->getX(), rsu->getY());
	rsuIndexText->setText(MainWindow::cstringDigits[rsu->getSelfIndex()]);
	rsuIndexText->setFont(timesFont);
	rsuIndexText->setColor(Qt::darkGreen);
	rsuInnerCircle = new QCPItemEllipse(ui->customPlot);
	rsuInnerCircle->topLeft->setCoords(rsu->getX() - RSU::r, rsu->getY() + RSU::r);
	rsuInnerCircle->bottomRight->setCoords(rsu->getX() + RSU::r, rsu->getY() - RSU::r);
	rsuInnerCircle->setPen(solidPen);
	rsuOutterCircle = new QCPItemEllipse(ui->customPlot);
	rsuOutterCircle->topLeft->setCoords(rsu->getX() - RSU::R, rsu->getY() + RSU::R);
	rsuOutterCircle->bottomRight->setCoords(rsu->getX() + RSU::R, rsu->getY() - RSU::R);
	rsuOutterCircle->setPen(dashPen);
	solidPen.setColor(Qt::darkCyan);
	dashPen.setColor(Qt::darkCyan);

	ui->customPlot->replot();
}

void MainWindow::scheduleEvent()
{
	if (running == 0)
		return;
	int replot = 0;
	while (!solution.eventQ.empty() && solution.eventQ.top().first == solution.eventSequence)
	{
		EventInfo eventInfo = solution.eventQ.top().second;
		int indicator = eventInfo.indicator;
		solution.eventQ.pop();
		int _replot = solution.handleEvent((indicator & 0xffff0000) >> 16, indicator & 0xffff, eventInfo.data);
		if (_replot & 2)
		{
			RSU *rsu = &solution.rsu;
			QCPItemText *uavIndexText = new QCPItemText(ui->customPlot);
			uavIndexText->setPositionAlignment(Qt::AlignmentFlag::AlignCenter);
			uavIndexText->position->setCoords(rsu->getX(), rsu->getY());
			uavIndexText->setText(MainWindow::cstringDigits[uavIndices.size()]);
			uavIndexText->setFont(timesFont);
			uavIndexText->setColor(Qt::darkCyan);
			QCPItemEllipse *uavInnerCircle = new QCPItemEllipse(ui->customPlot);
			uavInnerCircle->topLeft->setCoords(rsu->getX() - UAV::r, rsu->getY() + UAV::r);
			uavInnerCircle->bottomRight->setCoords(rsu->getX() + UAV::r, rsu->getY() - UAV::r);
			uavInnerCircle->setPen(solidPen);
			QCPItemEllipse *uavOutterCircle = new QCPItemEllipse(ui->customPlot);
			uavOutterCircle->topLeft->setCoords(rsu->getX() - UAV::R, rsu->getY() + UAV::R);
			uavOutterCircle->bottomRight->setCoords(rsu->getX() + UAV::R, rsu->getY() - UAV::R);
			uavOutterCircle->setPen(dashPen);
			uavIndices.push_back(uavIndexText);
			uavInnerCircles.push_back(uavInnerCircle);
			uavOutterCircles.push_back(uavOutterCircle);
		}
		replot |= _replot;
	}
	if (replot > 0)
	{
		for (size_t k = 0; k < solution.uavs.size(); ++k)
		{
			UAV &uav = solution.uavs[k];
			QCPItemText *&index = uavIndices[k];
			QCPItemEllipse *&inner = uavInnerCircles[k];
			QCPItemEllipse *&outter = uavOutterCircles[k];
			index->position->setCoords(uav.getX(), uav.getY());
			inner->topLeft->setCoords(uav.getX() - UAV::r, uav.getY() + UAV::r);
			inner->bottomRight->setCoords(uav.getX() + UAV::r, uav.getY() - UAV::r);
			outter->topLeft->setCoords(uav.getX() - UAV::R, uav.getY() + UAV::R);
			outter->bottomRight->setCoords(uav.getX() + UAV::R, uav.getY() - UAV::R);
		}
	}
	if (elapsed > 0)
	{
		replot = 1;
		// update vehicles' positions and pass data points to graphs
		double drivingPeriod = static_cast<double>(elapsed) / 10.0;
		QCPGraph *graph0 = ui->customPlot->graph(0);
		QVector<double> xCoord(numVehicle), yCoord(numVehicle);
		for (int i = 0; i < numVehicle; ++i)
		{
			vehicleTable[i].x += drivingPeriod * velocityTable[i] * cos(directionTable[i]);
			vehicleTable[i].y += drivingPeriod * velocityTable[i] * sin(directionTable[i]);
			if (vehicleTable[i].x < 0 || vehicleTable[i].x >= gX)
			{
				if (vehicleTable[i].y < 0 || vehicleTable[i].y >= gY)
					vehicleTable[i].x = gX - vehicleTable[i].x;
				else
					vehicleTable[i].x += vehicleTable[i].x < 0 ? gX : -gX;
				vehicleTable[i].y = gY - vehicleTable[i].y;
			}
			else if (vehicleTable[i].y < 0 || vehicleTable[i].y >= gY)
			{
				vehicleTable[i].y += vehicleTable[i].y < 0 ? gY : -gY;
				vehicleTable[i].x = gX - vehicleTable[i].x;
			}
			xCoord[i] = vehicleTable[i].x;
			yCoord[i] = vehicleTable[i].y;
		}
		graph0->setData(xCoord, yCoord);
	}
	if (!solution.eventQ.empty())
	{
		elapsed = solution.eventQ.top().first - solution.eventSequence;
		solution.eventSequence = solution.eventQ.top().first;
	}
	else
	{
		elapsed = 1;
		++solution.eventSequence;
	}
	if (replot > 0)
		ui->customPlot->replot();
}

void MainWindow::pauseEvent()
{
	running = 0;
	qDebug().nospace() << "pauseEvent()";
}

void MainWindow::continueEvent()
{
	running = 1;
	qDebug().nospace() << "continueEvent()";
}

