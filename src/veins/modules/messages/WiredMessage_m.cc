//
// Generated file, do not edit! Created by nedtool 5.0 from WiredMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "WiredMessage_m.h"

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

Register_Class(WiredMessage);

WiredMessage::WiredMessage(const char *name, int kind) : ::omnetpp::cPacket(name,kind)
{
    this->controlCode = 0;
    this->downloader = 0;
    this->contentSize = 0;
    this->curOffset = 0;
    this->startOffset = 0;
    this->endOffset = 0;
    this->bytesNum = 0;
}

WiredMessage::WiredMessage(const WiredMessage& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

WiredMessage::~WiredMessage()
{
}

WiredMessage& WiredMessage::operator=(const WiredMessage& other)
{
    if (this==&other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void WiredMessage::copy(const WiredMessage& other)
{
    this->controlCode = other.controlCode;
    this->downloader = other.downloader;
    this->contentSize = other.contentSize;
    this->curOffset = other.curOffset;
    this->startOffset = other.startOffset;
    this->endOffset = other.endOffset;
    this->bytesNum = other.bytesNum;
    this->position = other.position;
    this->speed = other.speed;
    this->neighbors = other.neighbors;
}

void WiredMessage::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->controlCode);
    doParsimPacking(b,this->downloader);
    doParsimPacking(b,this->contentSize);
    doParsimPacking(b,this->curOffset);
    doParsimPacking(b,this->startOffset);
    doParsimPacking(b,this->endOffset);
    doParsimPacking(b,this->bytesNum);
    doParsimPacking(b,this->position);
    doParsimPacking(b,this->speed);
    doParsimPacking(b,this->neighbors);
}

void WiredMessage::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->controlCode);
    doParsimUnpacking(b,this->downloader);
    doParsimUnpacking(b,this->contentSize);
    doParsimUnpacking(b,this->curOffset);
    doParsimUnpacking(b,this->startOffset);
    doParsimUnpacking(b,this->endOffset);
    doParsimUnpacking(b,this->bytesNum);
    doParsimUnpacking(b,this->position);
    doParsimUnpacking(b,this->speed);
    doParsimUnpacking(b,this->neighbors);
}

int WiredMessage::getControlCode() const
{
    return this->controlCode;
}

void WiredMessage::setControlCode(int controlCode)
{
    this->controlCode = controlCode;
}

int WiredMessage::getDownloader() const
{
    return this->downloader;
}

void WiredMessage::setDownloader(int downloader)
{
    this->downloader = downloader;
}

int WiredMessage::getContentSize() const
{
    return this->contentSize;
}

void WiredMessage::setContentSize(int contentSize)
{
    this->contentSize = contentSize;
}

int WiredMessage::getCurOffset() const
{
    return this->curOffset;
}

void WiredMessage::setCurOffset(int curOffset)
{
    this->curOffset = curOffset;
}

int WiredMessage::getStartOffset() const
{
    return this->startOffset;
}

void WiredMessage::setStartOffset(int startOffset)
{
    this->startOffset = startOffset;
}

int WiredMessage::getEndOffset() const
{
    return this->endOffset;
}

void WiredMessage::setEndOffset(int endOffset)
{
    this->endOffset = endOffset;
}

int WiredMessage::getBytesNum() const
{
    return this->bytesNum;
}

void WiredMessage::setBytesNum(int bytesNum)
{
    this->bytesNum = bytesNum;
}

Coord& WiredMessage::getPosition()
{
    return this->position;
}

void WiredMessage::setPosition(const Coord& position)
{
    this->position = position;
}

Coord& WiredMessage::getSpeed()
{
    return this->speed;
}

void WiredMessage::setSpeed(const Coord& speed)
{
    this->speed = speed;
}

NeighborItems& WiredMessage::getNeighbors()
{
    return this->neighbors;
}

void WiredMessage::setNeighbors(const NeighborItems& neighbors)
{
    this->neighbors = neighbors;
}

class WiredMessageDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    WiredMessageDescriptor();
    virtual ~WiredMessageDescriptor();

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

Register_ClassDescriptor(WiredMessageDescriptor);

WiredMessageDescriptor::WiredMessageDescriptor() : omnetpp::cClassDescriptor("WiredMessage", "omnetpp::cPacket")
{
    propertynames = nullptr;
}

WiredMessageDescriptor::~WiredMessageDescriptor()
{
    delete[] propertynames;
}

bool WiredMessageDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<WiredMessage *>(obj)!=nullptr;
}

const char **WiredMessageDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *WiredMessageDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int WiredMessageDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 10+basedesc->getFieldCount() : 10;
}

unsigned int WiredMessageDescriptor::getFieldTypeFlags(int field) const
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
    };
    return (field>=0 && field<10) ? fieldTypeFlags[field] : 0;
}

const char *WiredMessageDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "controlCode",
        "downloader",
        "contentSize",
        "curOffset",
        "startOffset",
        "endOffset",
        "bytesNum",
        "position",
        "speed",
        "neighbors",
    };
    return (field>=0 && field<10) ? fieldNames[field] : nullptr;
}

int WiredMessageDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='c' && strcmp(fieldName, "controlCode")==0) return base+0;
    if (fieldName[0]=='d' && strcmp(fieldName, "downloader")==0) return base+1;
    if (fieldName[0]=='c' && strcmp(fieldName, "contentSize")==0) return base+2;
    if (fieldName[0]=='c' && strcmp(fieldName, "curOffset")==0) return base+3;
    if (fieldName[0]=='s' && strcmp(fieldName, "startOffset")==0) return base+4;
    if (fieldName[0]=='e' && strcmp(fieldName, "endOffset")==0) return base+5;
    if (fieldName[0]=='b' && strcmp(fieldName, "bytesNum")==0) return base+6;
    if (fieldName[0]=='p' && strcmp(fieldName, "position")==0) return base+7;
    if (fieldName[0]=='s' && strcmp(fieldName, "speed")==0) return base+8;
    if (fieldName[0]=='n' && strcmp(fieldName, "neighbors")==0) return base+9;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *WiredMessageDescriptor::getFieldTypeString(int field) const
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
        "Coord",
        "Coord",
        "NeighborItems",
    };
    return (field>=0 && field<10) ? fieldTypeStrings[field] : nullptr;
}

const char **WiredMessageDescriptor::getFieldPropertyNames(int field) const
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

const char *WiredMessageDescriptor::getFieldProperty(int field, const char *propertyname) const
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

int WiredMessageDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    WiredMessage *pp = (WiredMessage *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string WiredMessageDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    WiredMessage *pp = (WiredMessage *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getControlCode());
        case 1: return long2string(pp->getDownloader());
        case 2: return long2string(pp->getContentSize());
        case 3: return long2string(pp->getCurOffset());
        case 4: return long2string(pp->getStartOffset());
        case 5: return long2string(pp->getEndOffset());
        case 6: return long2string(pp->getBytesNum());
        case 7: {std::stringstream out; out << pp->getPosition(); return out.str();}
        case 8: {std::stringstream out; out << pp->getSpeed(); return out.str();}
        case 9: {std::stringstream out; out << pp->getNeighbors(); return out.str();}
        default: return "";
    }
}

bool WiredMessageDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    WiredMessage *pp = (WiredMessage *)object; (void)pp;
    switch (field) {
        case 0: pp->setControlCode(string2long(value)); return true;
        case 1: pp->setDownloader(string2long(value)); return true;
        case 2: pp->setContentSize(string2long(value)); return true;
        case 3: pp->setCurOffset(string2long(value)); return true;
        case 4: pp->setStartOffset(string2long(value)); return true;
        case 5: pp->setEndOffset(string2long(value)); return true;
        case 6: pp->setBytesNum(string2long(value)); return true;
        default: return false;
    }
}

const char *WiredMessageDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        case 7: return omnetpp::opp_typename(typeid(Coord));
        case 8: return omnetpp::opp_typename(typeid(Coord));
        case 9: return omnetpp::opp_typename(typeid(NeighborItems));
        default: return nullptr;
    };
}

void *WiredMessageDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    WiredMessage *pp = (WiredMessage *)object; (void)pp;
    switch (field) {
        case 7: return (void *)(&pp->getPosition()); break;
        case 8: return (void *)(&pp->getSpeed()); break;
        case 9: return (void *)(&pp->getNeighbors()); break;
        default: return nullptr;
    }
}


