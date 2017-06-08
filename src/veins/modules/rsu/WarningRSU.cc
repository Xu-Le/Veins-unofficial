//
// Copyright (C) 2016 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "veins/modules/rsu/WarningRSU.h"

Define_Module(WarningRSU);

extern std::map<int /* GUID */, LAddress::L3Type> warningMaxDelayHelper;

void WarningRSU::initialize(int stage)
{
    BaseRSU::initialize(stage);

    if (stage == 0)
    {
        guidUsedTime = par("guidUsedTime").doubleValue();
        laneId = par("laneId").stringValue();

        if (sendWarnings)
        {
            initializeWarningPlanList(par("warningPlan").xmlValue());

            if ( !warningPlanList.empty() )
            {
                callWarningEvt = new cMessage("call warning evt", RSUMessageKinds::WARNING_NOTIFY_EVT);
                recycleGUIDEvt = new cMessage("recycle guid evt", RSUMessageKinds::RECYCLE_GUID_EVT);
                scheduleAt(warningPlanList.front().first, callWarningEvt);
            }
            else
            {
                callWarningEvt = nullptr;
                recycleGUIDEvt = nullptr;
            }
        }
        else
        {
            callWarningEvt = nullptr;
            recycleGUIDEvt = nullptr;
        }
    }
}


void WarningRSU::finish()
{
    // clear containers
    guidUsed.clear();
    warningPlanList.clear();

    // delete handle self message, it is safe to delete nullptr
    if (callWarningEvt != nullptr && callWarningEvt->isScheduled())
        cancelAndDelete(callWarningEvt);
    else
        delete callWarningEvt;
    if (recycleGUIDEvt != nullptr && recycleGUIDEvt->isScheduled())
        cancelAndDelete(recycleGUIDEvt);
    else
        delete recycleGUIDEvt;

    BaseRSU::finish();
}

void WarningRSU::handleSelfMsg(cMessage *msg)
{
    switch (msg->getKind())
    {
        case RSUMessageKinds::WARNING_NOTIFY_EVT:
        {
            callWarning(warningPlanList.front().second);
            warningPlanList.pop_front();
            if (!warningPlanList.empty())
                scheduleAt(warningPlanList.front().first, callWarningEvt);
            break;
        }
        case RSUMessageKinds::RECYCLE_GUID_EVT:
        {
            RoutingUtils::recycleGUID(guidUsed.front());
            guidUsed.pop_front();
            break;
        }
        default:
            BaseRSU::handleSelfMsg(msg);
    }
}

void WarningRSU::decorateWSM(WaveShortMessage *wsm)
{
    BaseRSU::decorateWSM(wsm);
}

void WarningRSU::onRouting(RoutingMessage *routingMsg)
{
    EV << "WarningRSUs don't react to routing messages since they don't help routings.\n";
}

void WarningRSU::onContent(ContentMessage* contentMsg)
{
    EV << "WarningRSUs don't react to content messages since they don't provide content service.\n";
}

void WarningRSU::onData(DataMessage* dataMsg)
{
    EV << "WarningRSUs don't react to data messages since they don't provide content service.\n";
}

void WarningRSU::callWarning(double distance)
{
    bubble("call warning");
    // create message by factory method
    WaveShortMessage *wsm = prepareWSM("warning", warningLengthBits, type_CCH, warningPriority, -1);
    if (wsm == nullptr) return;
    WarningMessage *warningMessage = dynamic_cast<WarningMessage*>(wsm);
    // handle utils about message's GUID
    int guid = RoutingUtils::generateGUID();
    EV << "GUID = " << guid << ", distance = " << distance << std::endl;
    guidUsed.push_back(guid);
    scheduleAt(simTime() + guidUsedTime, recycleGUIDEvt);
    // set necessary routing info
    warningMessage->setGUID(guid);
    warningMessage->setSenderPos(verticalPoint);
    warningMessage->setDirection(false);
    double farthestDistance = distance; // the farthest target vehicle for warning message in ROI
    warningMessage->setFarthestDistance(farthestDistance);
    warningMessage->setLaneId(laneId.c_str());
    warningMessage->setHopCount(0);
    decorateWSM(warningMessage);
    sendWSM(warningMessage);

    ++WarningStatisticCollector::globalNotifications;
    // calculate the number of target vehicles

    bool plusX = true;
    Coord xMin, xMax;
    LAddress::L3Type farthestOne = -1;
    if ( whichSide == (int)SideDirection::EAST_SIDE || whichSide == (int)SideDirection::SOUTH_SIDE )
        plusX = true;
    else // ( whichSide == (int)SideDirection::WEST_SIDE || whichSide == (int)SideDirection::NORTH_SIDE )
        plusX = false;
    if (distFromRoadhead <= farthestDistance)
    {
        EV << "distFromRoadhead = " << distFromRoadhead << " <= " << farthestDistance << std::endl;
        if (plusX) // fromRoadhead.x <= toRoadhead.x
        {
            xMin = fromRoadhead;
            xMax = verticalPoint;
            xMax.x -= 1.0; // avoid count self vehicle
        }
        else // fromRoadhead.x > toRoadhead.x
        {
            xMin = verticalPoint;
            xMin.x += 1.0; // avoid count self vehicle
            xMax = fromRoadhead;
        }
    }
    else
    {
        EV << "distFromRoadhead = " << distFromRoadhead << " > " << farthestDistance << std::endl;
        Coord farthestPos;
        farthestPos.x = verticalPoint.x + farthestDistance/distFromRoadhead * (fromRoadhead.x - verticalPoint.x);
        farthestPos.y = verticalPoint.y + farthestDistance/distFromRoadhead * (fromRoadhead.y - verticalPoint.y);
        farthestPos.z = verticalPoint.z + farthestDistance/distFromRoadhead * (fromRoadhead.z - verticalPoint.z);
        EV << "farthestPos.x: " << farthestPos.x << ", farthestPos.y: " << farthestPos.y << ", farthestPos.z: " << farthestPos.z << std::endl;
        if (plusX) // fromRoadhead.x <= toRoadhead.x
        {
            xMin = farthestPos;
            xMax = verticalPoint;
            xMax.x -= 1.0; // avoid count self vehicle
        }
        else // fromRoadhead.x > toRoadhead.x
        {
            xMin = verticalPoint;
            xMin.x += 1.0; // avoid count self vehicle
            xMax = farthestPos;
        }
    }

    WarningStatisticCollector::globalTargets += targetVehicles(plusX, xMin, xMax, farthestOne);

    warningMaxDelayHelper.insert(std::pair<int, LAddress::L3Type>(guid, farthestOne));
}

void WarningRSU::initializeWarningPlanList(cXMLElement* xmlConfig)
{
    if (xmlConfig == 0)
        throw cRuntimeError("No routing plan configuration file specified.");

    cXMLElementList planList = xmlConfig->getElementsByTagName("WarningPlan");

    if (planList.empty())
        EV << "No warning plan configuration items specified.\n";

    for (cXMLElementList::iterator iter = planList.begin(); iter != planList.end(); ++iter)
    {
        cXMLElement *warningPlan = *iter;

        const char* name = warningPlan->getAttribute("type");

        if (atoi(name) == myAddr)
        {
            cXMLElementList parameters = warningPlan->getElementsByTagName("parameter");

            ASSERT( parameters.size() % 2 == 0 );

            EV << logName() << "'s warning plan list as follows:\n";

            for (size_t i = 0; i < parameters.size(); i += 2)
            {
                double simtime = 0.0;
                double distance = 0.0;

                const char *name = parameters[i]->getAttribute("name");
                const char *type = parameters[i]->getAttribute("type");
                const char *value = parameters[i]->getAttribute("value");
                if (name == 0 || type == 0 || value == 0)
                    throw cRuntimeError("Invalid parameter, could not find name, type or value.");

                if (strcmp(name, "simtime") == 0 && strcmp(type, "double") == 0)
                    simtime = atof(value);
                else
                    throw cRuntimeError("Invalid parameter, name or type undefined.");

                name = parameters[i+1]->getAttribute("name");
                type = parameters[i+1]->getAttribute("type");
                value = parameters[i+1]->getAttribute("value");
                if (name == 0 || type == 0 || value == 0)
                    throw cRuntimeError("Invalid parameter, could not find name, type or value.");

                if (strcmp(name, "distance") == 0 && strcmp(type, "double") == 0)
                    distance = atof(value);
                else
                    throw cRuntimeError("Invalid parameter, name or type undefined.");

                if (distance < 0)
                    throw cRuntimeError("Distance configure of RSUs can only be positive.");

                EV << "    simtime: " << simtime << ", distance: " << distance << std::endl;
                warningPlanList.push_back(std::pair<double, double>(simtime, distance));
            }

            break;
        }
    }
}

long WarningRSU::targetVehicles(bool plusX, Coord xMin, Coord xMax, LAddress::L3Type& farthestOne)
{
    MobilityObserver *mobilityObserver = MobilityObserver::Instance();
    ASSERT( mobilityObserver->globalPosition.size() == mobilityObserver->globalSpeed.size() );

    long targetNumber = 0;
    double distToSourceX = 0.0;

    EV << "target vehicles are: ";

    std::map<LAddress::L3Type, Coord>::iterator iterPos, iterSpeed;
    for (iterPos = mobilityObserver->globalPosition.begin(), iterSpeed = mobilityObserver->globalSpeed.begin();
         iterPos != mobilityObserver->globalPosition.end() && iterSpeed != mobilityObserver->globalSpeed.end();
         ++iterPos, ++iterSpeed)
    {
        if ( (plusX && iterSpeed->second.x >= 0) || (!plusX && iterSpeed->second.x < 0) )
        {
            if ( iterPos->second.x >= xMin.x && iterPos->second.x <= xMax.x && RoutingUtils::distanceToLine(iterPos->second, xMin, xMax) < 14.0 )
            {
                if (plusX && distToSourceX < xMax.x - iterPos->second.x)
                {
                    distToSourceX = xMax.x - iterPos->second.x;
                    farthestOne = iterPos->first;
                }
                else if (!plusX && distToSourceX < iterPos->second.x - xMin.x)
                {
                    distToSourceX = iterPos->second.x - xMin.x;
                    farthestOne = iterPos->first;
                }
                EV << ' ' << iterPos->first;
                ++targetNumber;
            }
        }
    }

    EV << ", the number of them is " << targetNumber << ", the farthest one is " << farthestOne << std::endl;

    return targetNumber;
}
