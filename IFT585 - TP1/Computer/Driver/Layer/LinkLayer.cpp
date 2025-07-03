#include "LinkLayer.h"
#include "../NetworkDriver.h"

#include "../../../General/Configuration.h"
#include "../../../General/Logger.h"

#include <iostream>
#include <functional>
#include <map>
#include <set>


LinkLayer::LinkLayer(NetworkDriver* driver, const Configuration& config)
    : m_driver(driver)
    , m_address(config)
    , m_receivingQueue(config.get(Configuration::LINK_LAYER_RECEIVING_BUFFER_SIZE))
    , m_sendingQueue(config.get(Configuration::LINK_LAYER_SENDING_BUFFER_SIZE))
    , m_maximumBufferedFrameCount(config.get(Configuration::LINK_LAYER_MAXIMUM_BUFFERED_FRAME))
    , m_transmissionTimeout(config.get(Configuration::LINK_LAYER_TIMEOUT))
    , m_executeReceiving(false)
    , m_executeSending(false)
{
    m_maximumSequence = m_maximumBufferedFrameCount * 2 - 1;
    m_ackTimeout = m_transmissionTimeout / 4;
    m_timers = std::make_unique<Timer>();
    Logger log(std::cout);
    log << "Max seq: " << m_maximumSequence << std::endl;
    log << "Max outstanding: " << m_maximumBufferedFrameCount << std::endl;
}

LinkLayer::~LinkLayer()
{
    stop();
    m_driver = nullptr;
}

const MACAddress& LinkLayer::getMACAddress() const
{
    return m_address;
}

// Demarre les fils d'execution pour l'envoi et la reception des trames
void LinkLayer::start()
{
    stop();

    m_timers->start();

    m_executeReceiving = true;
    m_receiverThread = std::thread(&LinkLayer::receiverCallback, this);

    m_executeSending = true;
    m_senderThread = std::thread(&LinkLayer::senderCallback, this);
}

// Arrete les fils d'execution pour l'envoi et la reception des trames
void LinkLayer::stop()
{
    m_timers->stop();

    m_executeReceiving = false;
    if (m_receiverThread.joinable())
    {
        m_receiverThread.join();
    }

    m_executeSending = false;
    if (m_senderThread.joinable())
    {
        m_senderThread.join();
    }
}

// Indique vrai si on peut envoyer des donnees dans le buffer de sortie, faux si le buffer est plein
bool LinkLayer::canSendData(const Frame& data) const
{
    return m_sendingQueue.canWrite<Frame>(data);
}

// Indique vrai si des donnees sont disponibles dans le buffer d'entree, faux s'il n'y a rien
bool LinkLayer::dataReceived() const
{
    return m_receivingQueue.canRead<Frame>();
}

// Indique vrai s'il y a des donnees dans le buffer de sortie
bool LinkLayer::dataReady() const
{
    return m_sendingQueue.canRead<Frame>();
}

// Recupere la prochaine donnee du buffer de sortie
Frame LinkLayer::getNextData()
{
    return m_sendingQueue.pop<Frame>();
}

// Envoit une trame dans le buffer de sortie
// Cette fonction retourne faux si la trame n'a pas ete envoyee. Ce cas arrive seulement si le programme veut se terminer.
// Fait de l'attente active jusqu'a ce qu'il puisse envoyer la trame sinon.
bool LinkLayer::sendFrame(const Frame& frame)
{
    while (m_executeSending)
    {
        if (canSendData(frame))
        {
            // Vous pouvez décommenter ce code pour avoir plus de détails dans la console lors de l'exécution
            Logger log(std::cout);
            if (frame.Size == FrameType::NAK)
            {
                log << frame.Source << " : Sending NAK  to " << frame.Destination << " : " << frame.Ack << std::endl;
            }
            else if (frame.Size == FrameType::ACK)
            {
                log << frame.Source << " : Sending ACK  to " << frame.Destination << " : " << frame.Ack << std::endl;
            }
            else
            {
                log << frame.Source << " : Sending DATA to " << frame.Destination << " : " << frame.NumberSeq << std::endl;
            }
            m_sendingQueue.push(frame);
            return true;
        }
    }
    return false;
}

// Recupere le prochain evenement de communication a gerer pour l'envoi de donnees
LinkLayer::Event LinkLayer::getNextSendingEvent()
{
    std::lock_guard<std::mutex> lock(m_sendEventMutex);
    if (m_sendingEventQueue.size() > 0)
    {
        Event ev = m_sendingEventQueue.front();
        m_sendingEventQueue.pop();
        return ev;
    }
    return Event::Invalid();
}

// Recupere le prochain evenement de communication a gerer pour la reception de donnees
LinkLayer::Event LinkLayer::getNextReceivingEvent()
{
    std::lock_guard<std::mutex> lock(m_receiveEventMutex);
    if (m_receivingEventQueue.size() > 0)
    {
        Event ev = m_receivingEventQueue.front();
        m_receivingEventQueue.pop();
        return ev;
    }
    return Event::Invalid();
}

// Indique si la valeur est comprise entre first et last de facon circulaire
bool LinkLayer::between(NumberSequence value, NumberSequence first, NumberSequence last) const
{
    // Value is between first and last, circular style
    return ((first <= value) && (value < last)) || ((last < first) && (first <= value)) || ((value < last) && (last < first));
}

// Envoit un evenement de communication pour indiquer a l'envoi d'envoyer un ACK
// L'evenement contiendra l'adresse a qui il faut envoyer un ACK et le numero du ACK
void LinkLayer::sendAck(const MACAddress& to, NumberSequence ackNumber)
{
    Event ev = Event::Invalid();
    ev.Type = EventType::SEND_ACK_REQUEST;
    ev.Number = ackNumber;
    ev.Address = to;
    std::lock_guard<std::mutex> lock(m_sendEventMutex);
    m_sendingEventQueue.push(ev);
}

// Envoit un evenement de communication pour indiquer a l'envoi d'envoyer un NAK
// L'evenement contiendra l'adresse a qui il faut envoyer un ACK et le numero du NAK
void LinkLayer::sendNak(const MACAddress& to, NumberSequence nakNumber)
{
    Event ev = Event::Invalid();
    ev.Type = EventType::SEND_NAK_REQUEST;
    ev.Number = nakNumber;
    ev.Address = to;
    std::lock_guard<std::mutex> lock(m_sendEventMutex);
    m_sendingEventQueue.push(ev);
}

// Envoit un evenement de communication pour indiquer a l'envoi qu'on a recu une trame avec potentiellement un ACK (piggybacking)
// L'evenement contiendra l'adresse d'ou provient l'information, le numero du ACK et le prochain ACK qu'on devrait nous-meme envoyer (pour le piggybacking)
void LinkLayer::notifyACK(const Frame& frame, NumberSequence piggybackAck)
{
    Event ev = Event::Invalid();
    ev.Type = EventType::ACK_RECEIVED;
    ev.Number = frame.Ack;
    ev.Address = frame.Source;
    ev.Next = piggybackAck;
    std::lock_guard<std::mutex> lock(m_sendEventMutex);
    m_sendingEventQueue.push(ev);
}

// Envoit un evenement de communication pour indiquer a l'envoi qu'on a recu un NAK
// L'evenement contiendra l'adresse d'ou provient l'information et le numero du NAK
void LinkLayer::notifyNAK(const Frame& frame)
{
    Event ev = Event::Invalid();
    ev.Type = EventType::NAK_RECEIVED;
    ev.Number = frame.Ack;
    ev.Address = frame.Source;
    std::lock_guard<std::mutex> lock(m_sendEventMutex);
    m_sendingEventQueue.push(ev);
}

// Envoit un evenement de communication pour indiquer au recepteur qu'on a atteint un timeout pour un ACK
// L'evenement contiendra le numero du Timer qui est arrive a echeance et le numero de la trame associe au Timer
void LinkLayer::ackTimeout(size_t timerID, NumberSequence numberData)
{
    Event ev;
    ev.Type = EventType::ACK_TIMEOUT;
    ev.Number = numberData;
    ev.TimerID = timerID;
    std::lock_guard<std::mutex> guard(m_receiveEventMutex);
    m_receivingEventQueue.push(ev);
}

// Envoit un evenement de communication pour indiquer a l'envoi qu'on n'a aps recu de reponse a un envoit et qu'il faut reenvoyer la trame
// L'evenement contiendra le numero de la trame et le numero du Timer qui est arrive a echeance
void LinkLayer::transmissionTimeout(size_t timerID, NumberSequence numberData)
{
    Event ev;
    ev.Type = EventType::SEND_TIMEOUT;
    ev.Number = numberData;
    ev.TimerID = timerID;
    std::lock_guard<std::mutex> guard(m_sendEventMutex);
    m_sendingEventQueue.push(ev);
}


// Demarre un nouveau Timer d'attente pour l'envoi a nouveau d'une trame
// La methode retourne le numero du Timer qui vient d'etre demarre. Cette valeur doit etre garder pour pouvoir retrouver quel evenement y sera associe lorsque
// le timer arrivera a echeance
size_t LinkLayer::startTimeoutTimer(NumberSequence numberData)
{
    return m_timers->addTimer(m_transmissionTimeout, std::bind(&LinkLayer::transmissionTimeout, this, std::placeholders::_1, std::placeholders::_2), numberData);
}

// Demarre un nouveau Timer pour l'envoi d'un ACK, pour garantir un niveau de service minimal dans une communication unidirectionnelle
// Retourne le numero du Timer qui vient d'etre demarre. La methode prend en parametre le numero actuel du Timer de ACK afin de le redemarrer s'il existe encore
size_t LinkLayer::startAckTimer(size_t existingTimerID, NumberSequence ackNumber)
{
    if (!m_timers->restartTimer(existingTimerID, ackNumber))
    {
        return m_timers->addTimer(m_ackTimeout, std::bind(&LinkLayer::ackTimeout, this, std::placeholders::_1, std::placeholders::_2), ackNumber);
    }
    return existingTimerID;
}

// Envoit un evenement de communication pour indiquer a la fonction de reception qu'une ACK vient d'etre envoyer (en piggybacking) et 
// qu'on n'a pas besoin d'envoyer le ACK en attente
void LinkLayer::notifyStopAckTimers(const MACAddress& to)
{
    Event ev;
    ev.Type = EventType::STOP_ACK_TIMER_REQUEST;
    ev.Address = to;
    std::lock_guard<std::mutex> guard(m_receiveEventMutex);
    m_receivingEventQueue.push(ev);
}

// Arrete le Timer de ACK avec le TimerID specifie
void LinkLayer::stopAckTimer(size_t timerID)
{
    m_timers->removeTimer(timerID);
}

// Indique s'il y a assez de place dans le buffer de reception pour recevoir des donnees de la couche physique
bool LinkLayer::canReceiveDataFromPhysicalLayer(const Frame& data) const
{
    return m_receivingQueue.canWrite<Frame>(data);
}

// Recoit des donnees de la couche physique
void LinkLayer::receiveData(Frame data)
{
    // Si la couche est pleine, la trame est perdue. Elle devra etre envoye a nouveau par l'emetteur
    if (canReceiveDataFromPhysicalLayer(data))
    {
        // Est-ce que la trame reçue est pour nous?
        if (data.Destination == m_address || data.Destination.isMulticast())
        {
            m_receivingQueue.push(data);
        }
    }
}

// Fonction qui retourne l'adresse MAC du destinataire d'un packet particulier de la couche Reseau.
// Dans la realite, cette fonction ferait un lookup dans une table a partir des adresses IP pour recupere les addresse MAC.
// Ici, on utilise directement seulement les adresse MAC.
MACAddress LinkLayer::arp(const Packet& packet) const
{
    return packet.Destination;
}

// Fonction qui fait l'envoi des trames et qui gere la fenetre d'envoi
void LinkLayer::senderCallback()
{
    const NumberSequence window_size = m_maximumBufferedFrameCount;
    const NumberSequence max_seq = m_maximumSequence;
    NumberSequence send_base = 0;
    NumberSequence next_seq = 0;

    std::map<NumberSequence, Frame> out_buf;
    std::map<NumberSequence, size_t> frame_timers;
    std::set<NumberSequence> acked;

    while (m_executeSending)
    {
        // Handle events (ACK, NAK, TIMEOUT)
        while (true)
        {
            Event ev = getNextSendingEvent();
            if (ev.Type == EventType::INVALID) break;

            switch (ev.Type)
            {
                case EventType::SEND_ACK_REQUEST:
                {
                    Frame ackFrame;
                    ackFrame.Source = m_address;
                    ackFrame.Destination = ev.Address;
                    ackFrame.NumberSeq = 0;
                    ackFrame.Ack = ev.Number;
                    ackFrame.Size = FrameType::ACK;
                    sendFrame(ackFrame);
                    break;
                }
                case EventType::SEND_NAK_REQUEST:
                {
                    Frame nakFrame;
                    nakFrame.Source = m_address;
                    nakFrame.Destination = ev.Address;
                    nakFrame.NumberSeq = 0;
                    nakFrame.Ack = ev.Number;
                    nakFrame.Size = FrameType::NAK;
                    sendFrame(nakFrame);
                    break;
                }
                case EventType::NAK_RECEIVED:
                {
	                    NumberSequence seq = ev.Number;

	                    if (out_buf.count(seq))
	                    {
	                        if (frame_timers.count(seq))
	                            stopAckTimer(frame_timers[seq]);
	                        frame_timers[seq] = startTimeoutTimer(seq);
	                        sendFrame(out_buf[seq]);
	                    }
	                    break;
                }
                case EventType::SEND_TIMEOUT:
                {
                    NumberSequence seq = ev.Number;
                    if (out_buf.count(seq))
                    {
                        if (frame_timers.count(seq))
                            stopAckTimer(frame_timers[seq]);
                        frame_timers[seq] = startTimeoutTimer(seq);
                        sendFrame(out_buf[seq]);
                    }
                    break;
                }
                case EventType::ACK_RECEIVED:
                {
                    NumberSequence ack = ev.Number;
                    // Mark this frame as acknowledged
					acked.insert(ack);

					// Slide send_base forward for all consecutive acked frames
					while (acked.count(send_base))
					{
					    // Stop timer and remove frame for this sequence number
					    if (frame_timers.count(send_base))
					    {
					        stopAckTimer(frame_timers[send_base]);
					        frame_timers.erase(send_base);
					    }
					    out_buf.erase(send_base);
					    acked.erase(send_base);
					    send_base = (send_base + 1) % (max_seq + 1);
					}
                    break;
                }
                default:
                    break;
            }
        }

        // Only send if window is not full
        while (m_driver->getNetworkLayer().dataReady() &&
               ((next_seq - send_base + max_seq + 1) % (max_seq + 1)) < window_size)
        {
            Packet packet = m_driver->getNetworkLayer().getNextData();
            Frame frame;
            frame.Destination = arp(packet);
            frame.Source = m_address;
            frame.NumberSeq = next_seq;
            frame.Data = Buffering::pack<Packet>(packet);
            frame.Size = (uint16_t)frame.Data.size();

            // Piggyback ACK if pending
			std::lock_guard<std::mutex> lock(m_ackMutex);
			if (!m_pendingAck.empty())
			{
				std::pair<MACAddress, NumberSequence> ack = m_pendingAck.front();
				m_pendingAck.pop();
				frame.Ack = ack.second;
				notifyStopAckTimers(frame.Destination);
				m_ackTimerId = 0; // Stop the ACK timer since we piggybacked
			}
			else
			{
				frame.Ack = NO_ACK;
			}

            out_buf[next_seq] = frame;
            frame_timers[next_seq] = startTimeoutTimer(next_seq);
            sendFrame(frame);

            next_seq = (next_seq + 1) % (max_seq + 1);
        }
    }
}

void LinkLayer::receiverCallback()
{
    const NumberSequence window_size = m_maximumBufferedFrameCount;
    const NumberSequence max_seq = m_maximumSequence;
    NumberSequence recv_base = 0;

    std::map<NumberSequence, Frame> in_buf;
    std::set<NumberSequence> arrived;

    while (m_executeReceiving)
    {
        // Handle events (ACK timeout, STOP ACK timer)
        while (true)
        {
            Event ev = getNextReceivingEvent();
            if (ev.Type == EventType::INVALID) break;

            switch (ev.Type)
            {
                case EventType::STOP_ACK_TIMER_REQUEST:
                    stopAckTimer(ev.TimerID);
                    break;
                case EventType::ACK_TIMEOUT:
                {
                    std::unique_lock<std::mutex> lock(m_ackMutex);
                    if (!m_pendingAck.empty())
                    {
						std::pair<MACAddress, NumberSequence> ack = m_pendingAck.front();
						m_pendingAck.pop();
                        sendAck(ack.first, ack.second);
                        m_ackTimerId = 0;
                    }
                    break;
                }
                default:
                    break;
            }
        }

        if (m_receivingQueue.canRead<Frame>())
        {
            Frame frame = m_receivingQueue.pop<Frame>();
            if (frame.Size == FrameType::ACK)
            {
                notifyACK(frame, frame.Ack);
                continue;
            }
            else if (frame.Size == FrameType::NAK)
            {
                notifyNAK(frame);
                continue;
            }

            NumberSequence seq = frame.NumberSeq;
            bool in_window = between(seq, recv_base, (recv_base + window_size) % (max_seq + 1));

            if (in_window)
            {
                // Always ACK every in-window frame, even if duplicate
                {
					std::lock_guard<std::mutex> lock(m_ackMutex);
					m_pendingAck.push(std::make_pair(frame.Source, seq));
                    m_ackTimerId = startAckTimer(m_ackTimerId, seq);
                    //sendAck(frame.Source, seq);
                }

                // If not already received, buffer it
                if (arrived.find(seq) == arrived.end())
                {
                    in_buf[seq] = frame;
                    arrived.insert(seq);
                }

                // Deliver in-order frames to upper layer
                while (arrived.find(recv_base) != arrived.end())
                {
                    Packet p = Buffering::unpack<Packet>(in_buf[recv_base].Data);
                    m_driver->getNetworkLayer().receiveData(p);
                    in_buf.erase(recv_base);
                    arrived.erase(recv_base);
                    recv_base = (recv_base + 1) % (max_seq + 1);
                }
            }
            else
            {
                // Out of window, send NAK for expected
                sendNak(frame.Source, recv_base);
            }

            // Handle piggybacked ACK
            if (frame.Ack != NO_ACK)
            {
                notifyACK(frame, frame.Ack);
            }
        }
    }
}

