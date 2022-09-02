#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <cstdint>
#include <string.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <pthread.h>
#include <mutex>
#include <vector>
#include <condition_variable>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/core/noncopyable.hpp>

#include "crc.h"
#include "datatypes.h"
namespace vesc_can_driver
{

  typedef std::vector<uint8_t> Buffer;
  typedef std::pair<Buffer::iterator, Buffer::iterator> BufferRange;
  typedef std::pair<Buffer::const_iterator, Buffer::const_iterator> BufferRangeConst;
  class VescCanInterface : private boost::noncopyable
  {

  private:
    struct sockaddr_can m_addr;
    struct can_frame m_recvframe;
    struct ifreq m_ifr;
    int m_socket;
    int m_sock_err;
    int m_nbytes;
    int m_dev_id;
    char m_rx_buffer[80];
    int m_rx_last_buffer_id;
    int m_commands_send;
    uint16_t m_rxbuf_len;
    uint16_t m_crc_check;
    std::chrono::time_point<std::chrono::steady_clock> m_last_rx_packet = std::chrono::steady_clock::now();
    int process_rx_buffer();
    int handle_packet();
    uint32_t genEId(uint32_t, uint32_t);
    bool update_thread_run_;
    bool rx_thread_run_;
    uint32_t state_request_millis = 1; //maybe signed?
    std::mutex status_mutex_;
    std::mutex m_tx_mutex;
    pthread_t rx_thread_handle_;
    pthread_t update_thread_handle_;
    std::condition_variable status_cv_;
    static void *rx_thread_helper(void *context) 
    {
      return ((VescCanInterface *) context)->rx_thread();
    }
    static void *update_thread_helper(void *context) 
    {
      return ((VescCanInterface *) context)->update_thread();
    }

  public:
    VescStatusStruct status_;
    int m_pack;
    uint8_t host_id;

  public:
    VescCanInterface();
    ~VescCanInterface();
    void stop(); 
    bool send(can_frame *frame); 
    void start(const std::string &int_name, uint32_t id); 
    void *update_thread();
    void *rx_thread();

    void get_status(VescStatusStruct *status);
    void wait_for_status(VescStatusStruct *status);
    void requestState();
    void requestFWVersion();
    void setDutyCycle(double);
    void setCurrent(double);
    void setBrake(double);
    void setSpeed(double);
    void setPosition(double);

  };
}


void bldc_buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void bldc_buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void bldc_buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void bldc_buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void bldc_buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
void bldc_buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);
int16_t bldc_buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t bldc_buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t bldc_buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t bldc_buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float bldc_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
float bldc_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);
