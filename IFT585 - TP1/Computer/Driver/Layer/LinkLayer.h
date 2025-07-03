#ifndef _COMPUTER_DRIVER_LAYER_LINK_LAYER_H_
#define _COMPUTER_DRIVER_LAYER_LINK_LAYER_H_

#include "DataType.h"
#include "../../../DataStructures/CircularQueue.h"
#include "../../../DataStructures/DataBuffer.h"
#include "../../../DataStructures/MACAddress.h"
#include "../../../General/Timer.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <queue>
#include <map>
#include <set>
#include <mutex>
#include <thread>

class Configuration;
class NetworkDriver;

class LinkLayer
{
private:

 
    enum class EventType
    {
        INVALID,
        ACK_TIMEOUT,
        SEND_TIMEOUT,
        ACK_RECEIVED,
        NAK_RECEIVED,
        SEND_ACK_REQUEST,
        SEND_NAK_REQUEST,
        STOP_ACK_TIMER_REQUEST,
    };

    struct Event
    {
        EventType Type = EventType::INVALID;
        size_t Number = 0;
        size_t TimerID = 0;
        MACAddress Address;
        NumberSequence Next = 0;

        static Event Invalid()
        {
            return Event();
        }
    };

    NetworkDriver* m_driver;
    std::unique_ptr<Timer> m_timers;
    std::mutex m_frameMapMutex;

    MACAddress m_address;

    NumberSequence m_maximumSequence;
    NumberSequence m_maximumBufferedFrameCount;

    std::chrono::milliseconds m_transmissionTimeout;
    std::chrono::milliseconds m_ackTimeout;

    std::queue<Event> m_receivingEventQueue;
    std::queue<Event> m_sendingEventQueue;

    CircularQueue m_sendingQueue;
    CircularQueue m_receivingQueue;

    std::atomic<bool> m_executeReceiving;
    std::atomic<bool> m_executeSending;

    std::mutex m_mutex;                 
    std::mutex m_receiveEventMutex;
    std::mutex m_sendEventMutex;

    std::mutex m_eventQueueMutex;        
    std::mutex m_framesSentMutex;        
    std::mutex m_timerAssociationMutex;  

    std::thread m_senderThread;
    std::thread m_receiverThread;

    std::map<NumberSequence, Frame> m_FramesSent;
    std::map<size_t, Frame> m_EventFrameAssociation;
    NumberSequence m_sendBase = 0; 
    NumberSequence m_nextID = 0;
    std::mutex m_idMutex;

    void receiverCallback();
    void senderCallback();

    bool canSendData(const Frame& data) const;
    bool between(NumberSequence value, NumberSequence first, NumberSequence last) const;

    void sendAck(const MACAddress& to, NumberSequence ackNumber);
    void sendNak(const MACAddress& to, NumberSequence nakNumber);
    bool sendFrame(const Frame& frame);

    void notifyNAK(const Frame& frame);
    void notifyACK(const Frame& frame, NumberSequence piggybackAck);

    void transmissionTimeout(size_t timerID, NumberSequence numberData);
    void ackTimeout(size_t timerID, NumberSequence numberData);

    size_t startAckTimer(size_t existingTimerID, NumberSequence ackNumber);
    void stopAckTimer(size_t timerID);
    void notifyStopAckTimers(const MACAddress& to);

    size_t startTimeoutTimer(NumberSequence number);

    Event getNextSendingEvent();
    Event getNextReceivingEvent();

    MACAddress arp(const Packet& p) const;
    bool canReceiveDataFromPhysicalLayer(const Frame& data) const;
    bool isSendEventQueueEmpty() ;
    bool isReceiveEventQueueEmpty() ;

public:
    LinkLayer(NetworkDriver* driver, const Configuration& config);
    ~LinkLayer();

    const MACAddress& getMACAddress() const;

    void start();
    void stop();

    bool dataReady() const;
    Frame getNextData();

    bool dataReceived() const;
    void receiveData(Frame data);
};

#endif //_COMPUTER_DRIVER_LAYER_LINK_LAYER_H_
