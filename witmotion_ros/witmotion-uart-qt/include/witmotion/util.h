#ifndef WITMOTION_UTIL
#define WITMOTION_UTIL
#include "witmotion/types.h"

namespace witmotion
{

class witmotion_typed_packets
{
private:
    witmotion_datapacket array[32];
public:
    witmotion_datapacket& operator[](const witmotion_packet_id id);
};

class witmotion_typed_bytecounts
{
private:
    size_t array[32];
public:
    witmotion_typed_bytecounts();
    size_t& operator[](const witmotion_packet_id id);
};

uint8_t witmotion_output_frequency(const int hertz);

uint8_t witmotion_baud_rate(const QSerialPort::BaudRate rate);

bool id_registered(const size_t id);

/* COMPONENT DECODERS */
float decode_acceleration(const int16_t* value);
float decode_angular_velocity(const int16_t* value);
float decode_angle(const int16_t* value);
float decode_temperature(const int16_t* value);
float decode_orientation(const int16_t* value);
void decode_gps_coord(const int32_t* value,
                      double& deg,
                      double& min);

/* PACKET DECODERS */
void decode_realtime_clock(const witmotion_datapacket& packet,
                           uint8_t& year,
                           uint8_t& month,
                           uint8_t& day,
                           uint8_t& hour,
                           uint8_t& minute,
                           uint8_t& second,
                           uint16_t& millisecond);
void decode_accelerations(const witmotion_datapacket& packet,
                          float& x,
                          float& y,
                          float& z,
                          float& t);
void decode_angular_velocities(const witmotion_datapacket& packet,
                               float& x,
                               float& y,
                               float& z,
                               float& t);
void decode_angles(const witmotion_datapacket& packet,
                   float& roll,
                   float& pitch,
                   float& yaw,
                   float& t);
void decode_magnetometer(const witmotion_datapacket& packet,
                         float& x,
                         float& y,
                         float& z,
                         float& t);
void decode_altimeter(const witmotion_datapacket& packet,
                      double& pressure,
                      double& height);
void decode_gps(const witmotion_datapacket& packet,
                double& longitude_deg,
                double& longitude_min,
                double& latitude_deg,
                double& latitude_min);
void decode_gps_ground_speed(const witmotion_datapacket& packet,
                             float& altitude,
                             float& angular_velocity,
                             double& ground_speed);
void decode_orientation(const witmotion_datapacket& packet,
                        float& x,
                        float& y,
                        float& z,
                        float& w);
void decode_gps_accuracy(const witmotion_datapacket& packet,
                         size_t& satellites,
                         float& local_accuracy,
                         float& horizontal_accuracy,
                         float& vertical_accuracy);
}
#endif
