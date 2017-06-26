//
// Generated file, do not edit! Created by nedtool 5.0 from ContentMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "ContentMessage_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: no doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: no doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp


// forward
template<typename T, typename A>
std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec);

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// operator<< for std::vector<T>
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

Register_Class(ContentMessage);

ContentMessage::ContentMessage(const char *name, int kind) : ::WaveShortMessage(name,kind)
{
    this->controlCode = 0;
    this->receiver = 0;
    this->downloader = 0;
    this->contentSize = 0;
    this->receivedOffset = 0;
    this->consumedOffset = 0;
    this->consumingRate = 0;
}

ContentMessage::ContentMessage(const ContentMessage& other) : ::WaveShortMessage(other)
{
    copy(other);
}

ContentMessage::~ContentMessage()
{
}

ContentMessage& ContentMessage::operator=(const ContentMessage& other)
{
    if (this==&other) return *this;
    ::WaveShortMessage::operator=(other);
    copy(other);
    return *this;
}

void ContentMessage::copy(const ContentMessage& other)
{
    this->controlCode = other.controlCode;
    this->receiver = other.receiver;
    this->downloader = other.downloader;
    this->contentSize = other.contentSize;
    this->receivedOffset = other.receivedOffset;
    this->consumedOffset = other.consumedOffset;
    this->consumingRate = other.consumingRate;
    this->lackOffset = other.lackOffset;
    this->scheme = other.scheme;
    this->position = other.position;
    this->speed = other.speed;
    this->neighbors = other.neighbors;
}

void ContentMessage::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::WaveShortMessage::parsimPack(b);
    doParsimPacking(b,this->controlCode);
    doParsimPacking(b,this->receiver);
    doParsimPacking(b,this->downloader);
    doParsimPacking(b,this->contentSize);
    doParsimPacking(b,this->receivedOffset);
    doParsimPacking(b,this->consumedOffset);
    doParsimPacking(b,this->consumingRate);
    doParsimPacking(b,this->lackOffset);
    doParsimPacking(b,this->scheme);
    doParsimPacking(b,this->position);
    doParsimPacking(b,this->speed);
    doParsimPacking(b,this->neighbors);
}

void ContentMessage::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::WaveShortMessage::parsimUnpack(b);
    doParsimUnpacking(b,this->controlCode);
    doParsimUnpacking(b,this->receiver);
    doParsimUnpacking(b,this->downloader);
    doParsimUnpacking(b,this->contentSize);
    doParsimUnpacking(b,this->receivedOffset);
    doParsimUnpacking(b,this->consumedOffset);
    doParsimUnpacking(b,this->consumingRate);
    doParsimUnpacking(b,this->lackOffset);
    doParsimUnpacking(b,this->scheme);
    doParsimUnpacking(b,this->position);
    doParsimUnpacking(b,this->speed);
    doParsimUnpacking(b,this->neighbors);
}

int ContentMessage::getControlCode() const
{
    return this->controlCode;
}

void ContentMessage::setControlCode(int controlCode)
{
    this->controlCode = controlCode;
}

int ContentMessage::getReceiver() const
{
    return this->receiver;
}

void ContentMessage::setReceiver(int receiver)
{
    this->receiver = receiver;
}

int ContentMessage::getDownloader() const
{
    return this->downloader;
}

void ContentMessage::setDownloader(int downloader)
{
    this->downloader = downloader;
}

int ContentMessage::getContentSize() const
{
    return this->contentSize;
}

void ContentMessage::setContentSize(int contentSize)
{
    this->contentSize = contentSize;
}

int ContentMessage::getReceivedOffset() const
{
    return this->receivedOffset;
}

void ContentMessage::setReceivedOffset(int receivedOffset)
{
    this->receivedOffset = receivedOffset;
}

int ContentMessage::getConsumedOffset() const
{
    return this->consumedOffset;
}

void ContentMessage::setConsumedOffset(int consumedOffset)
{
    this->consumedOffset = consumedOffset;
}

int ContentMessage::getConsumingRate() const
{
    return this->consumingRate;
}

void ContentMessage::setConsumingRate(int consumingRate)
{
    this->consumingRate = consumingRate;
}

Segment& ContentMessage::getLackOffset()
{
    return this->lackOffset;
}

void ContentMessage::setLackOffset(const Segment& lackOffset)
{
    this->lackOffset = lackOffset;
}

SchemeItems& ContentMessage::getScheme()
{
    return this->scheme;
}

void ContentMessage::setScheme(const SchemeItems& scheme)
{
    this->scheme = scheme;
}

Coord& ContentMessage::getPosition()
{
    return this->position;
}

void ContentMessage::setPosition(const Coord& position)
{
    this->position = position;
}

Coord& ContentMessage::getSpeed()
{
    return this->speed;
}

void ContentMessage::setSpeed(const Coord& speed)
{
    this->speed = speed;
}

NeighborItems& ContentMessage::getNeighbors()
{
    return this->neighbors;
}

void ContentMessage::setNeighbors(const NeighborItems& neighbors)
{
    this->neighbors = neighbors;
}

class ContentMessageDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    ContentMessageDescriptor();
    virtual ~ContentMessageDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyname) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyname) const override;
    virtual int getFieldArraySize(void *object, int field) const override;

    virtual std::string getFieldValueAsString(void *object, int field, int i) const override;
    virtual bool setFieldValueAsString(void *object, int field, int i, const char *value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual void *getFieldStructValuePointer(void *object, int field, int i) const override;
};

Register_ClassDescriptor(ContentMessageDescriptor);

ContentMessageDescriptor::ContentMessageDescriptor() : omnetpp::cClassDescriptor("ContentMessage", "WaveShortMessage")
{
    propertynames = nullptr;
}

ContentMessageDescriptor::~ContentMessageDescriptor()
{
    delete[] propertynames;
}

bool ContentMessageDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<ContentMessage *>(obj)!=nullptr;
}

const char **ContentMessageDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *ContentMessageDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int ContentMessageDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 12+basedesc->getFieldCount() : 12;
}

unsigned int ContentMessageDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeFlags(field);
        field -= basedesc->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
    };
    return (field>=0 && field<12) ? fieldTypeFlags[field] : 0;
}

const char *ContentMessageDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "controlCode",
        "receiver",
        "downloader",
        "contentSize",
        "receivedOffset",
        "consumedOffset",
        "consumingRate",
        "lackOffset",
        "scheme",
        "position",
        "speed",
        "neighbors",
    };
    return (field>=0 && field<12) ? fieldNames[field] : nullptr;
}

int ContentMessageDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='c' && strcmp(fieldName, "controlCode")==0) return base+0;
    if (fieldName[0]=='r' && strcmp(fieldName, "receiver")==0) return base+1;
    if (fieldName[0]=='d' && strcmp(fieldName, "downloader")==0) return base+2;
    if (fieldName[0]=='c' && strcmp(fieldName, "contentSize")==0) return base+3;
    if (fieldName[0]=='r' && strcmp(fieldName, "receivedOffset")==0) return base+4;
    if (fieldName[0]=='c' && strcmp(fieldName, "consumedOffset")==0) return base+5;
    if (fieldName[0]=='c' && strcmp(fieldName, "consumingRate")==0) return base+6;
    if (fieldName[0]=='l' && strcmp(fieldName, "lackOffset")==0) return base+7;
    if (fieldName[0]=='s' && strcmp(fieldName, "scheme")==0) return base+8;
    if (fieldName[0]=='p' && strcmp(fieldName, "position")==0) return base+9;
    if (fieldName[0]=='s' && strcmp(fieldName, "speed")==0) return base+10;
    if (fieldName[0]=='n' && strcmp(fieldName, "neighbors")==0) return base+11;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *ContentMessageDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "int",
        "int",
        "int",
        "int",
        "int",
        "int",
        "Segment",
        "SchemeItems",
        "Coord",
        "Coord",
        "NeighborItems",
    };
    return (field>=0 && field<12) ? fieldTypeStrings[field] : nullptr;
}

const char **ContentMessageDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldPropertyNames(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *ContentMessageDescriptor::getFieldProperty(int field, const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldProperty(field, propertyname);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int ContentMessageDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    ContentMessage *pp = (ContentMessage *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string ContentMessageDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    ContentMessage *pp = (ContentMessage *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getControlCode());
        case 1: return long2string(pp->getReceiver());
        case 2: return long2string(pp->getDownloader());
        case 3: return long2string(pp->getContentSize());
        case 4: return long2string(pp->getReceivedOffset());
        case 5: return long2string(pp->getConsumedOffset());
        case 6: return long2string(pp->getConsumingRate());
        case 7: {std::stringstream out; out << pp->getLackOffset(); return out.str();}
        case 8: {std::stringstream out; out << pp->getScheme(); return out.str();}
        case 9: {std::stringstream out; out << pp->getPosition(); return out.str();}
        case 10: {std::stringstream out; out << pp->getSpeed(); return out.str();}
        case 11: {std::stringstream out; out << pp->getNeighbors(); return out.str();}
        default: return "";
    }
}

bool ContentMessageDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    ContentMessage *pp = (ContentMessage *)object; (void)pp;
    switch (field) {
        case 0: pp->setControlCode(string2long(value)); return true;
        case 1: pp->setReceiver(string2long(value)); return true;
        case 2: pp->setDownloader(string2long(value)); return true;
        case 3: pp->setContentSize(string2long(value)); return true;
        case 4: pp->setReceivedOffset(string2long(value)); return true;
        case 5: pp->setConsumedOffset(string2long(value)); return true;
        case 6: pp->setConsumingRate(string2long(value)); return true;
        default: return false;
    }
}

const char *ContentMessageDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        case 7: return omnetpp::opp_typename(typeid(Segment));
        case 8: return omnetpp::opp_typename(typeid(SchemeItems));
        case 9: return omnetpp::opp_typename(typeid(Coord));
        case 10: return omnetpp::opp_typename(typeid(Coord));
        case 11: return omnetpp::opp_typename(typeid(NeighborItems));
        default: return nullptr;
    };
}

void *ContentMessageDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    ContentMessage *pp = (ContentMessage *)object; (void)pp;
    switch (field) {
        case 7: return (void *)(&pp->getLackOffset()); break;
        case 8: return (void *)(&pp->getScheme()); break;
        case 9: return (void *)(&pp->getPosition()); break;
        case 10: return (void *)(&pp->getSpeed()); break;
        case 11: return (void *)(&pp->getNeighbors()); break;
        default: return nullptr;
    }
}


