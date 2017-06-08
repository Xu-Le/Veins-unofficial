//
// Generated file, do not edit! Created by nedtool 5.0 from veins/modules/messages/BeaconMessage.msg.
//

#ifndef __BEACONMESSAGE_M_H
#define __BEACONMESSAGE_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0500
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
// }}

/**
 * Class generated from <tt>veins/modules/messages/BeaconMessage.msg:26</tt> by nedtool.
 * <pre>
 * packet BeaconMessage extends WaveShortMessage
 * {
 *     Coord senderFrom;  // The tail of road where sender is driving from
 *     Coord senderTo;    // The head of road where sender is driving to
 * }
 * </pre>
 */
class BeaconMessage : public ::WaveShortMessage
{
  protected:
    Coord senderFrom;
    Coord senderTo;

  private:
    void copy(const BeaconMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const BeaconMessage&);

  public:
    BeaconMessage(const char *name=nullptr, int kind=0);
    BeaconMessage(const BeaconMessage& other);
    virtual ~BeaconMessage();
    BeaconMessage& operator=(const BeaconMessage& other);
    virtual BeaconMessage *dup() const {return new BeaconMessage(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b);

    // field getter/setter methods
    virtual Coord& getSenderFrom();
    virtual const Coord& getSenderFrom() const {return const_cast<BeaconMessage*>(this)->getSenderFrom();}
    virtual void setSenderFrom(const Coord& senderFrom);
    virtual Coord& getSenderTo();
    virtual const Coord& getSenderTo() const {return const_cast<BeaconMessage*>(this)->getSenderTo();}
    virtual void setSenderTo(const Coord& senderTo);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const BeaconMessage& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, BeaconMessage& obj) {obj.parsimUnpack(b);}


#endif // ifndef __BEACONMESSAGE_M_H

