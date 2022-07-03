#include "witmotion/wt31n-uart.h"

namespace witmotion
{
namespace wt31n
{

const std::set<witmotion_packet_id> QWitmotionWT31NSensor::registered_types =
{
    pidAcceleration,
    pidAngles
};

void QWitmotionWT31NSensor::Calibrate()
{
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridCalibrate;
    config_packet.setting.raw[0] = 0x01;
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    config_packet.address_byte = ridSaveSettings;
    config_packet.setting.raw[0] = 0x00;
    emit SendConfig(config_packet);
}

void QWitmotionWT31NSensor::SetBaudRate(const QSerialPort::BaudRate &rate)
{
    if(!((rate == QSerialPort::Baud9600) || (rate == QSerialPort::Baud115200)))
    {
        emit ErrorOccurred("Only 9600 or 115200 baud rates are supported for WT31N!");
        return;
    }
    witmotion_config_packet config_packet;
    config_packet.header_byte = WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridPortBaudRate;
    port_rate = rate;
    config_packet.setting.raw[0] = witmotion_baud_rate(port_rate);
    config_packet.setting.raw[1] = 0x00;
    emit SendConfig(config_packet);
    config_packet.address_byte = ridOutputFrequency;
    config_packet.setting.raw[0] = (port_rate == QSerialPort::Baud9600) ? witmotion_output_frequency(2) : witmotion_output_frequency(10);
    emit SendConfig(config_packet);
    config_packet.address_byte = ridSaveSettings;
    config_packet.setting.raw[0] = 0x00;
    emit SendConfig(config_packet);
    ttyout << "Reopening port..." << endl;
    reader->Suspend();
    sleep(1);
    emit RunReader();
}

QWitmotionWT31NSensor::QWitmotionWT31NSensor(const QString device, const QSerialPort::BaudRate rate):
    QAbstractWitmotionSensorController(device, rate)
{
    ttyout << "Creating multithreaded interface for Witmotion WT31N IMU sensor connected to "
           << port_name
           << "at "
           << static_cast<int32_t>(port_rate)
           << endl;
}

const std::set<witmotion_packet_id> *QWitmotionWT31NSensor::RegisteredPacketTypes()
{
    return &registered_types;
}

void QWitmotionWT31NSensor::Start()
{
    ttyout << "Running reader thread" << endl;
    if(!((port_rate == QSerialPort::Baud9600) || (port_rate == QSerialPort::Baud115200)))
        emit ErrorOccurred("Only 9600 or 115200 baud rates are supported for WT31N!");
    else
        emit RunReader();
}

}
}
