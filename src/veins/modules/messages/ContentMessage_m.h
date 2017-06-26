//
// Generated file, do not edit! Created by nedtool 5.0 from ContentMessage.msg.
//

#ifndef __CONTENTMESSAGE_M_H
#define __CONTENTMESSAGE_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0500
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/modules/application/ContentUtils.h"

enum ContentMsgCC {
	CONTENT_REQUEST,       ///< content request;
	CONTENT_RESPONSE,      ///< RSU's response to content request;
	SCHEME_DISTRIBUTION,   ///< distribute current RSU's slot decision to relays;
	ACKNOWLEDGEMENT,       ///< acknowledgement received offset to RSU and relay at the end of each slot;
	CARRIER_SELECTION,     ///< selected to be carrier by the cooperative RSU;
	CARRIER_ENCOUNTER,     ///< carrier encounter the downloader its carried data belongs to;
	DOWNLOADING_COMPLETED, ///< downloading precess completion notification broadcast by downloader;
	LINK_BREAK_DIRECT,     ///< link break notify - the communication link between downloader and RSU will break soon;
	LINK_BREAK_DR,         ///< link break notify - the communication link between downloader and relay will break soon;
	LINK_BREAK_RR,         ///< link break notify - the communication link between relay and RSU will break soon;
	RELAY_DISCOVERY,       ///< cooperative RSU's discover notification sends to the first entering relay;
	DISCOVERY_RESPONSE,    ///< the first entering relay response to cooperative RSU's discover notification;
	STATUS_QUERY           ///< query downloading status of downloader.
};

class SchemeTuple
{
public:
	SchemeTuple() : slot(-1), receiver(-1), downloader(-1), amount(-1), _offset(), offset(&_offset) {}
	SchemeTuple(int s, int r, int d, int a) : slot(s), receiver(r), downloader(d), amount(a), _offset(), offset(&_offset) {}
	SchemeTuple(const SchemeTuple& rhs) : slot(rhs.slot), receiver(rhs.receiver), downloader(rhs.downloader), amount(rhs.amount)
	{
		_offset.assign(&rhs._offset);
		offset = &_offset;
	}

	SchemeTuple& operator=(const SchemeTuple& rhs)
	{
		if (this == &rhs)
			return *this;
		slot = rhs.slot;
		receiver = rhs.receiver;
		downloader = rhs.downloader;
		amount = rhs.amount;
		_offset.assign(&rhs._offset);
		offset = &_offset;
		return *this;
	}

	int slot;
	int receiver;
	int downloader;
	int amount;
	Segment _offset; ///< internal variable, head node of segment list, thus the whole list can be cleared when its destructor automatically called.
	Segment *offset; ///< external variable, use offset = offset->next to iterate the segment list.
};

typedef std::map<long /* addr */, std::list<SchemeTuple> > SchemeItems;
// }}

/**
 * Class generated from <tt>ContentMessage.msg:81</tt> by nedtool.
 * <pre>
 * packet ContentMessage extends WaveShortMessage
 * {
 *     int controlCode; // express which kind of control signaling message, see enum ContentMsgCC.
 *     int receiver;    // identifier of receiver's application
 *     int downloader;  // which downloader this content message aims to
 *     int contentSize; // content size of request large-volume file (controlCode: 0)
 *     int receivedOffset; // data amount received offset (controlCode: 2,7,8,9)
 *     int consumedOffset; // data amount consumed offset (controlCode: 2,7,8,9)
 *     int consumingRate;  // data amount consuming rate measured in Bytes each second (controlCode: 0)
 *     Segment lackOffset; // lacking data segment's offset (controlCode: 3)
 *     SchemeItems scheme; // transmitssion scheme each vehicle should obey (controlCode: 2)
 *     Coord position;  // position of the co-downloader (controlCode: 7,8,9)
 *     Coord speed;     // speed of the co-downloader (controlCode: 7,8,9)
 *     NeighborItems neighbors; // neighbors of the co-downloader (controlCode: 7,8,9)
 * }
 * </pre>
 */
class ContentMessage : public ::WaveShortMessage
{
  protected:
    int controlCode;
    int receiver;
    int downloader;
    int contentSize;
    int receivedOffset;
    int consumedOffset;
    int consumingRate;
    Segment lackOffset;
    SchemeItems scheme;
    Coord position;
    Coord speed;
    NeighborItems neighbors;

  private:
    void copy(const ContentMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const ContentMessage&);

  public:
    ContentMessage(const char *name=nullptr, int kind=0);
    ContentMessage(const ContentMessage& other);
    virtual ~ContentMessage();
    ContentMessage& operator=(const ContentMessage& other);
    virtual ContentMessage *dup() const {return new ContentMessage(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b);

    // field getter/setter methods
    virtual int getControlCode() const;
    virtual void setControlCode(int controlCode);
    virtual int getReceiver() const;
    virtual void setReceiver(int receiver);
    virtual int getDownloader() const;
    virtual void setDownloader(int downloader);
    virtual int getContentSize() const;
    virtual void setContentSize(int contentSize);
    virtual int getReceivedOffset() const;
    virtual void setReceivedOffset(int receivedOffset);
    virtual int getConsumedOffset() const;
    virtual void setConsumedOffset(int consumedOffset);
    virtual int getConsumingRate() const;
    virtual void setConsumingRate(int consumingRate);
    virtual Segment& getLackOffset();
    virtual const Segment& getLackOffset() const {return const_cast<ContentMessage*>(this)->getLackOffset();}
    virtual void setLackOffset(const Segment& lackOffset);
    virtual SchemeItems& getScheme();
    virtual const SchemeItems& getScheme() const {return const_cast<ContentMessage*>(this)->getScheme();}
    virtual void setScheme(const SchemeItems& scheme);
    virtual Coord& getPosition();
    virtual const Coord& getPosition() const {return const_cast<ContentMessage*>(this)->getPosition();}
    virtual void setPosition(const Coord& position);
    virtual Coord& getSpeed();
    virtual const Coord& getSpeed() const {return const_cast<ContentMessage*>(this)->getSpeed();}
    virtual void setSpeed(const Coord& speed);
    virtual NeighborItems& getNeighbors();
    virtual const NeighborItems& getNeighbors() const {return const_cast<ContentMessage*>(this)->getNeighbors();}
    virtual void setNeighbors(const NeighborItems& neighbors);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const ContentMessage& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, ContentMessage& obj) {obj.parsimUnpack(b);}


#endif // ifndef __CONTENTMESSAGE_M_H

