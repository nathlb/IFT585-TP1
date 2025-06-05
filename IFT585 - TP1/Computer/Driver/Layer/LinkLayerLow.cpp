#include "LinkLayerLow.h"

#include <iomanip>

#include "LinkLayer.h"
#include "../NetworkDriver.h"
#include "../../../DataStructures/DataBuffer.h"
#include "../../../General/Configuration.h"
#include "../../../General/Logger.h"


std::unique_ptr<DataEncoderDecoder> DataEncoderDecoder::CreateEncoderDecoder(const Configuration& config)
{
    int encoderDecoderConfig = config.get(Configuration::LINK_LAYER_LOW_DATA_ENCODER_DECODER);
    if (encoderDecoderConfig == 1)
    {
        return std::make_unique<HammingDataEncoderDecoder>();
    }
    else if (encoderDecoderConfig == 2)
    {
        return std::make_unique<CRCDataEncoderDecoder>();
    }
    else
    {
        return std::make_unique<PassthroughDataEncoderDecoder>();
    }
}


DynamicDataBuffer PassthroughDataEncoderDecoder::encode(const DynamicDataBuffer& data) const
{
    return data;
}

std::pair<bool, DynamicDataBuffer> PassthroughDataEncoderDecoder::decode(const DynamicDataBuffer& data) const
{
    return std::pair<bool, DynamicDataBuffer>(true, data);
}


//===================================================================
// Hamming Encoder decoder implementation
//===================================================================
HammingDataEncoderDecoder::HammingDataEncoderDecoder()
{
	// À faire TP
}

HammingDataEncoderDecoder::~HammingDataEncoderDecoder()
{
	// À faire TP
}

DynamicDataBuffer HammingDataEncoderDecoder::encode(const DynamicDataBuffer& data) const
{
	// À faire TP
     return data;
}

std::pair<bool, DynamicDataBuffer> HammingDataEncoderDecoder::decode(const DynamicDataBuffer& data) const
{
	// À faire TP
     return std::pair<bool, DynamicDataBuffer>(true, data);
}



//===================================================================
// CRC Encoder decoder implementation
//===================================================================
CRCDataEncoderDecoder::CRCDataEncoderDecoder()
{
}

CRCDataEncoderDecoder::~CRCDataEncoderDecoder()
{
}

uint8_t compute_crc8(const uint8_t* data, size_t length, uint8_t crc_generator = 0xEA)
{
	uint8_t crc = 0;
    for (size_t i = 0; i < length; i++)
    {
		crc ^= data[i];
		for (int j = 0; j < 8; j++)
		{
			if (crc & 0x80)
			{
				crc = (crc << 1) ^ crc_generator;
			}
			else
			{
				crc <<= 1;
			}
		}
    }
	return crc;
}


DynamicDataBuffer CRCDataEncoderDecoder::encode(const DynamicDataBuffer& data) const
{
	DynamicDataBuffer non_const_data = DynamicDataBuffer(data); // Créer un DynamicDataBuffer non const pour pouvoir lire les octets
	uint8_t* dataWithCRC = new uint8_t[data.size() + 1]; // Allouer un tableau de taille data.size() pour les données + crc

	non_const_data.readTo(dataWithCRC, 0, data.size()); // Copier les octets de data dans dataWithCRC

	uint8_t crc = compute_crc8(dataWithCRC, data.size()); // Calculer le CRC sur les données + l'octet de CRC

	dataWithCRC[data.size()] = crc; // Mettre le CRC à la fin des données
	DynamicDataBuffer dataWithCRCBuffer(data.size() + 1, dataWithCRC); // Créer un DynamicDataBuffer avec les données et le CRC
	delete[] dataWithCRC; // Libérer la mémoire allouée pour dataWithCRC

    return dataWithCRCBuffer;
}

std::pair<bool, DynamicDataBuffer> CRCDataEncoderDecoder::decode(const DynamicDataBuffer& data) const
{
	DynamicDataBuffer non_const_data = DynamicDataBuffer(data); // Créer un DynamicDataBuffer non const pour pouvoir lire les octets
	uint8_t* dataReceived = new uint8_t[data.size()]; // Allouer un tableau de taille data.size() pour les données reçues

	non_const_data.readTo(dataReceived, 0, data.size()); // Copier les octets de data dans dataReceived
	DynamicDataBuffer dataWithoutCRCBuffer = DynamicDataBuffer(data.size() - 1, dataReceived); // Créer un DynamicDataBuffer avec les données reçues sans le CRC

	uint8_t computedCRC = compute_crc8(dataReceived, data.size()); // Calculer le CRC sur les données
	bool isValid = (computedCRC == 0); // Vérifier si les données sont valides (CRC doit être 0)
	delete[] dataReceived; // Libérer la mémoire allouée pour dataReceived


    return std::pair<bool, DynamicDataBuffer>(isValid, dataWithoutCRCBuffer);
}


//===================================================================
// Network Driver Physical layer implementation
//===================================================================
LinkLayerLow::LinkLayerLow(NetworkDriver* driver, const Configuration& config)
    : m_driver(driver)
    , m_sendingBuffer(config.get(Configuration::LINK_LAYER_LOW_SENDING_BUFFER_SIZE))
    , m_receivingBuffer(config.get(Configuration::LINK_LAYER_LOW_RECEIVING_BUFFER_SIZE))
    , m_stopReceiving(true)
    , m_stopSending(true)
{
    m_encoderDecoder = DataEncoderDecoder::CreateEncoderDecoder(config);
}

LinkLayerLow::~LinkLayerLow()
{
    stop();
    m_driver = nullptr;
}

void LinkLayerLow::start()
{
    stop();

    start_receiving();
    start_sending();
}

void LinkLayerLow::stop()
{
    stop_receiving();
    stop_sending();
}

bool LinkLayerLow::dataReceived() const
{
    return m_receivingBuffer.canRead<DynamicDataBuffer>();
}

DynamicDataBuffer LinkLayerLow::encode(const DynamicDataBuffer& data) const
{
    return m_encoderDecoder->encode(data);
}

std::pair<bool, DynamicDataBuffer> LinkLayerLow::decode(const DynamicDataBuffer& data) const
{
    return m_encoderDecoder->decode(data);
}

void LinkLayerLow::start_receiving()
{
    m_stopReceiving = false;
    m_receivingThread = std::thread(&LinkLayerLow::receiving, this);
}

void LinkLayerLow::stop_receiving()
{
    m_stopReceiving = true;
    if (m_receivingThread.joinable())
    {
        m_receivingThread.join();
    }
}

void LinkLayerLow::start_sending()
{
    m_stopSending = false;
    m_sendingThread = std::thread(&LinkLayerLow::sending, this);
}

void LinkLayerLow::stop_sending()
{
    m_stopSending = true;
    if (m_sendingThread.joinable())
    {
        m_sendingThread.join();
    }
}


void LinkLayerLow::receiving()
{
    while (!m_stopReceiving)
    {
        if (dataReceived())
        {
            DynamicDataBuffer data = m_receivingBuffer.pop<DynamicDataBuffer>();
            std::pair<bool, DynamicDataBuffer> dataBuffer = decode(data);
            if (dataBuffer.first) // Les donnees recues sont correctes et peuvent etre utilisees
            {
                Frame frame = Buffering::unpack<Frame>(dataBuffer.second);
                m_driver->getLinkLayer().receiveData(frame);
            }
            else
            {
                // Les donnees recues sont corrompues et doivent etre delaissees
                Logger log(std::cout);
                log << m_driver->getMACAddress() << " : Corrupted data received" << std::endl;
            }
        }
    }
}

void LinkLayerLow::sending()
{
    while (!m_stopSending)
    {
        if (m_driver->getLinkLayer().dataReady())
        {
            Frame dataFrame = m_driver->getLinkLayer().getNextData();
            DynamicDataBuffer buffer = encode(Buffering::pack<Frame>(dataFrame));
            sendData(buffer);
        }
    }
}

void LinkLayerLow::receiveData(const DynamicDataBuffer& data)
{
    // Si le buffer est plein, on fait juste oublier les octets recus du cable
    // Sinon, on ajoute les octets au buffer
    if (m_receivingBuffer.canWrite<DynamicDataBuffer>(data))
    {
        m_receivingBuffer.push(data);
    }
    else
    {
        Logger log(std::cout);
        log << m_driver->getMACAddress() << " : Physical reception buffer full... data discarded" << std::endl;
    }
}

void LinkLayerLow::sendData(DynamicDataBuffer data)
{
    // Envoit une suite d'octet sur le cable connecte
    m_driver->sendToCard(data);
}