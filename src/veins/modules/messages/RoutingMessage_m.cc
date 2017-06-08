//
// Generated file, do not edit! Created by nedtool 5.0 from veins/modules/messages/RoutingMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "RoutingMessage_m.h"

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

Register_Class(RoutingMessage);

RoutingMessage::RoutingMessage(const char *name, int kind) : ::WaveShortMessage(name,kind)
{
    this->routingSuccess = false;
    this->backward = false;
    this->hopCount = 0;
    this->receiver = -1;
    this->nextHop = -1;
}

RoutingMessage::RoutingMessage(const RoutingMessage& other) : ::WaveShortMessage(other)
{
    copy(other);
}

RoutingMessage::~RoutingMessage()
{
}

RoutingMessage& RoutingMessage::operator=(const RoutingMessage& other)
{
    if (this==&other) return *this;
    ::WaveShortMessage::operator=(other);
    copy(other);
    return *this;
}

void RoutingMessage::copy(const RoutingMessage& other)
{
    this->routingSuccess = other.routingSuccess;
    this->backward = other.backward;
    this->hopCount = other.hopCount;
    this->receiver = other.receiver;
    this->nextHop = other.nextHop;
    this->hopInfo = other.hopInfo;
}

void RoutingMessage::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::WaveShortMessage::parsimPack(b);
    doParsimPacking(b,this->routingSuccess);
    doParsimPacking(b,this->backward);
    doParsimPacking(b,this->hopCount);
    doParsimPacking(b,this->receiver);
    doParsimPacking(b,this->nextHop);
    doParsimPacking(b,this->hopInfo);
}

void RoutingMessage::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::WaveShortMessage::parsimUnpack(b);
    doParsimUnpacking(b,this->routingSuccess);
    doParsimUnpacking(b,this->backward);
    doParsimUnpacking(b,this->hopCount);
    doParsimUnpacking(b,this->receiver);
    doParsimUnpacking(b,this->nextHop);
    doParsimUnpacking(b,this->hopInfo);
}

bool RoutingMessage::getRoutingSuccess() const
{
    return this->routingSuccess;
}

void RoutingMessage::setRoutingSuccess(bool routingSuccess)
{
    this->routingSuccess = routingSuccess;
}

bool RoutingMessage::getBackward() const
{
    return this->backward;
}

void RoutingMessage::setBackward(bool backward)
{
    this->backward = backward;
}

int RoutingMessage::getHopCount() const
{
    return this->hopCount;
}

void RoutingMessage::setHopCount(int hopCount)
{
    this->hopCount = hopCount;
}

long RoutingMessage::getReceiver() const
{
    return this->receiver;
}

void RoutingMessage::setReceiver(long receiver)
{
    this->receiver = receiver;
}

long RoutingMessage::getNextHop() const
{
    return this->nextHop;
}

void RoutingMessage::setNextHop(long nextHop)
{
    this->nextHop = nextHop;
}

HopItems& RoutingMessage::getHopInfo()
{
    return this->hopInfo;
}

void RoutingMessage::setHopInfo(const HopItems& hopInfo)
{
    this->hopInfo = hopInfo;
}

class RoutingMessageDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    RoutingMessageDescriptor();
    virtual ~RoutingMessageDescriptor();

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

Register_ClassDescriptor(RoutingMessageDescriptor);

RoutingMessageDescriptor::RoutingMessageDescriptor() : omnetpp::cClassDescriptor("RoutingMessage", "WaveShortMessage")
{
    propertynames = nullptr;
}

RoutingMessageDescriptor::~RoutingMessageDescriptor()
{
    delete[] propertynames;
}

bool RoutingMessageDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<RoutingMessage *>(obj)!=nullptr;
}

const char **RoutingMessageDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *RoutingMessageDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int RoutingMessageDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 6+basedesc->getFieldCount() : 6;
}

unsigned int RoutingMessageDescriptor::getFieldTypeFlags(int field) const
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
        FD_ISCOMPOUND,
    };
    return (field>=0 && field<6) ? fieldTypeFlags[field] : 0;
}

const char *RoutingMessageDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "routingSuccess",
        "backward",
        "hopCount",
        "receiver",
        "nextHop",
        "hopInfo",
    };
    return (field>=0 && field<6) ? fieldNames[field] : nullptr;
}

int RoutingMessageDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='r' && strcmp(fieldName, "routingSuccess")==0) return base+0;
    if (fieldName[0]=='b' && strcmp(fieldName, "backward")==0) return base+1;
    if (fieldName[0]=='h' && strcmp(fieldName, "hopCount")==0) return base+2;
    if (fieldName[0]=='r' && strcmp(fieldName, "receiver")==0) return base+3;
    if (fieldName[0]=='n' && strcmp(fieldName, "nextHop")==0) return base+4;
    if (fieldName[0]=='h' && strcmp(fieldName, "hopInfo")==0) return base+5;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *RoutingMessageDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "bool",
        "bool",
        "int",
        "long",
        "long",
        "HopItems",
    };
    return (field>=0 && field<6) ? fieldTypeStrings[field] : nullptr;
}

const char **RoutingMessageDescriptor::getFieldPropertyNames(int field) const
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

const char *RoutingMessageDescriptor::getFieldProperty(int field, const char *propertyname) const
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

int RoutingMessageDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    RoutingMessage *pp = (RoutingMessage *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string RoutingMessageDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    RoutingMessage *pp = (RoutingMessage *)object; (void)pp;
    switch (field) {
        case 0: return bool2string(pp->getRoutingSuccess());
        case 1: return bool2string(pp->getBackward());
        case 2: return long2string(pp->getHopCount());
        case 3: return long2string(pp->getReceiver());
        case 4: return long2string(pp->getNextHop());
        case 5: {std::stringstream out; out << pp->getHopInfo(); return out.str();}
        default: return "";
    }
}

bool RoutingMessageDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    RoutingMessage *pp = (RoutingMessage *)object; (void)pp;
    switch (field) {
        case 0: pp->setRoutingSuccess(string2bool(value)); return true;
        case 1: pp->setBackward(string2bool(value)); return true;
        case 2: pp->setHopCount(string2long(value)); return true;
        case 3: pp->setReceiver(string2long(value)); return true;
        case 4: pp->setNextHop(string2long(value)); return true;
        default: return false;
    }
}

const char *RoutingMessageDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        case 5: return omnetpp::opp_typename(typeid(HopItems));
        default: return nullptr;
    };
}

void *RoutingMessageDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    RoutingMessage *pp = (RoutingMessage *)object; (void)pp;
    switch (field) {
        case 5: return (void *)(&pp->getHopInfo()); break;
        default: return nullptr;
    }
}


