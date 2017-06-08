//
// Generated file, do not edit! Created by nedtool 5.0 from veins/modules/messages/PacketExpiredMessage.msg.
//

#ifndef __PACKETEXPIREDMESSAGE_M_H
#define __PACKETEXPIREDMESSAGE_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0500
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



/**
 * Class generated from <tt>veins/modules/messages/PacketExpiredMessage.msg:21</tt> by nedtool.
 * <pre>
 * //
 * // Defines a self control message that handle with packet expired event
 * //
 * message PacketExpiredMessage
 * {
 *     // indicates which packet expired
 *     int GUID = 0;
 * }
 * </pre>
 */
class PacketExpiredMessage : public ::omnetpp::cMessage
{
  protected:
    int GUID;

  private:
    void copy(const PacketExpiredMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const PacketExpiredMessage&);

  public:
    PacketExpiredMessage(const char *name=nullptr, int kind=0);
    PacketExpiredMessage(const PacketExpiredMessage& other);
    virtual ~PacketExpiredMessage();
    PacketExpiredMessage& operator=(const PacketExpiredMessage& other);
    virtual PacketExpiredMessage *dup() const {return new PacketExpiredMessage(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b);

    // field getter/setter methods
    virtual int getGUID() const;
    virtual void setGUID(int GUID);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const PacketExpiredMessage& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, PacketExpiredMessage& obj) {obj.parsimUnpack(b);}


#endif // ifndef __PACKETEXPIREDMESSAGE_M_H

