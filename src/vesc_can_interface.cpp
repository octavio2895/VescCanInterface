#include "vesc_can_interface.h"

namespace vesc_can_driver
{
  VescCanInterface::VescCanInterface()
  {
  }

  VescCanInterface::~VescCanInterface() 
  {
    // stop();
  }

  void *VescCanInterface::update_thread() 
  {
    printf("Starting update thread\n");
    std::chrono::time_point<std::chrono::steady_clock> last_fw_request = std::chrono::steady_clock::now();
    while (update_thread_run_) 
    {
      auto time = (const struct timespec) {0, state_request_millis * 1000000L};
      nanosleep(&time, nullptr);
      std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
      VESC_CONNECTION_STATE state;
      {
        std::unique_lock<std::mutex> lk(status_mutex_);
        state = status_.connection_state;
      }
      if (WAITING_FOR_FW >= state) 
      {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fw_request).count() >
            1000) 
        {
          printf("Requesting FW\n");
          last_fw_request = now;
          requestFWVersion();
          printf("Got FW: %d\n", status_.fw_version_major);
        }
        continue;
      }
      else if(status_.connection_state == vesc_can_driver::CONNECTED || status_.connection_state == vesc_can_driver::CONNECTED_INCOMPATIBLE_FW) 
      {
        // requestState();
      }
    }
    printf("Joining Update Thread\n");
    return nullptr;
  }

  void *VescCanInterface::rx_thread() 
  {
    printf("Starting RX Thread \n");
    Buffer buffer;
    buffer.reserve(4096);
    {
      std::unique_lock<std::mutex> lk(status_mutex_);
      status_ = {0};
      status_.connection_state = vesc_can_driver::DISCONNECTED;
    }
    while (rx_thread_run_) 
    {
      std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
      if (m_socket < 0 || 
          m_sock_err < 0 ||
          std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_rx_packet).count() > 10000)
      if (m_socket < 0 || m_sock_err < 0)
      {
        {
          printf("Socker error\n");
          std::unique_lock<std::mutex> lk(status_mutex_);
          status_ = {0};
          status_.connection_state = vesc_can_driver::DISCONNECTED;
        }
        try 
        {
          rx_thread_run_ = 0;
          continue;
          //TODO reconnect();
        }
        catch (std::exception &e) 
        {
          // retry later
          sleep(1);
          continue;
        }
        {
          std::unique_lock<std::mutex> lk(status_mutex_);
          status_.connection_state = vesc_can_driver::VESC_CONNECTION_STATE::WAITING_FOR_FW;
        }
      }
      m_pack = handle_packet();
      if(m_pack < 0) return nullptr;
      // last_rx_packet = now;
    }
    close(m_socket);
    return nullptr;
  }

  int VescCanInterface::handle_packet()
  {
    m_nbytes = read(m_socket, &m_recvframe, sizeof(struct can_frame));
    if(m_nbytes < 0)
    {
      perror("can raw socket read");
      printf("Error reading\n");
      return -1;
    }
    if((m_recvframe.can_id & 0xF0000000) == 0x20000000)
    {
      printf("Error frame\n");
      printf("Id: %x\n",(int)m_recvframe.can_id);
      printf("Data: %x, %x, %x, %x, %x, %x, %x, %x\n",(int)m_recvframe.data[0],
                                 (int)m_recvframe.data[1],(int)m_recvframe.data[2],
                                (int)m_recvframe.data[3],(int)m_recvframe.data[4],
        (int)m_recvframe.data[5],(int)m_recvframe.data[6],(int)m_recvframe.data[7]);
      // m_sock_err = -1;
      // return -1;
    }
    m_last_rx_packet = std::chrono::steady_clock::now();
    int ind = 0;
    uint8_t recv_id = m_recvframe.can_id;
    uint8_t pack_id = m_recvframe.can_id >> 8;  

    switch (pack_id)
    {
      case vesc_can_driver::CAN_PACKET_FILL_RX_BUFFER:
        memcpy(m_rx_buffer + m_recvframe.data[0], m_recvframe.data + 1, 7);
        break;
      case vesc_can_driver::CAN_PACKET_FILL_RX_BUFFER_LONG:
        break;
      case vesc_can_driver::CAN_PACKET_PROCESS_RX_BUFFER:
        m_rx_last_buffer_id = m_recvframe.data[0];
        m_commands_send = m_recvframe.data[1];
        m_rxbuf_len = (uint16_t)m_recvframe.data[2]<<8 | (uint16_t)m_recvframe.data[3];
        m_crc_check = (uint16_t)m_recvframe.data[4]<<8 | (uint16_t)m_recvframe.data[5];
        if (crc16((unsigned char *)m_rx_buffer, m_rxbuf_len) == m_crc_check)
          process_rx_buffer();
        printf("GOOOOT\n");
        return vesc_can_driver::CAN_PACKET_PROCESS_RX_BUFFER;
      case vesc_can_driver::CAN_PACKET_PROCESS_SHORT_BUFFER:
        break;
      case vesc_can_driver::CAN_PACKET_STATUS:
        status_.speed_erpm = (double)bldc_buffer_get_int32(m_recvframe.data, &ind);
        status_.current_motor = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) / 10.0;
        status_.duty_cycle = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) / 1000.0;
        status_cv_.notify_all();
        break;
      
      case vesc_can_driver::CAN_PACKET_STATUS_2:
        status_.charge_drawn = (double)bldc_buffer_get_int32(m_recvframe.data, &ind) /1000.0;
        status_.charge_regen = (double)bldc_buffer_get_int32(m_recvframe.data, &ind) /1000.0;
        status_cv_.notify_all();
        break;

      case vesc_can_driver::CAN_PACKET_STATUS_3:
        status_.energy_drawn = (double)bldc_buffer_get_int32(m_recvframe.data, &ind) /1000.0;
        status_.energy_regen = (double)bldc_buffer_get_int32(m_recvframe.data, &ind) /1000.0;
        status_cv_.notify_all();
        break;
      
      case vesc_can_driver::CAN_PACKET_STATUS_4:
        status_.temperature_pcb = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /10.0;
        status_.temperature_motor = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /10.0;
        status_.current_input = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /10.0;
        status_.current_pid_position = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /50.0;
        status_cv_.notify_all();
        break;
      
      case vesc_can_driver::CAN_PACKET_STATUS_5:
        status_.tacho = (double)bldc_buffer_get_int32(m_recvframe.data, &ind);
        status_.voltage_input = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /10.0;
        status_.reserved = bldc_buffer_get_int16(m_recvframe.data, &ind);
        status_cv_.notify_all();
        break;

      case vesc_can_driver::CAN_PACKET_STATUS_6:
        status_.ext_adc1 = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /1000.0;
        status_.ext_adc2 = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /1000.0;
        status_.ext_adc3 = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /1000.0;
        status_.servo = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /100.0;
        status_cv_.notify_all();
        break;

      default:
        return 255;
    }
    return pack_id;
  }

  int VescCanInterface::process_rx_buffer()
  {
    int ind = 0;
    int comm_pack_id = m_rx_buffer[ind++];
    
    switch(comm_pack_id)
    {
      case vesc_can_driver::COMM_FW_VERSION:
        status_.fw_version_major = (uint8_t)m_rx_buffer[ind++];
        status_.fw_version_minor = (uint8_t)m_rx_buffer[ind++];
        char * _hw_name = m_rx_buffer + ind;
        char * pos = strchr(_hw_name, 0);
        int delta = pos - _hw_name;
        memcpy(status_.hw_name, _hw_name, pos - m_rx_buffer);
        status_.hw_name[delta + 1] = '\n';
        ind += delta+1;
        memcpy(status_.uuid, m_rx_buffer + ind, 12);
        ind += 12;
        status_.pairing_done = m_rx_buffer[ind++]; 
        status_.custom_config = m_rx_buffer[ind++];
        if (status_.fw_version_major == 5 && status_.fw_version_minor == 3) 
        {
          status_.phase_filters = m_rx_buffer[ind++];
          status_.qmlui_hw = m_rx_buffer[ind++];
          status_.qmlui_app = m_rx_buffer[ind];
        }
        if(status_.connection_state == vesc_can_driver::WAITING_FOR_FW || status_.connection_state == vesc_can_driver::DISCONNECTED)
          status_.connection_state = vesc_can_driver::CONNECTED;
        
        status_cv_.notify_all();
        break;
    }
    
    printf("FW: %d\n", status_.fw_version_major);
    return 0;
  }

  bool VescCanInterface::send(can_frame *send_frame) 
  {
    std::unique_lock<std::mutex> lk(m_tx_mutex);
    if (m_sock_err == 0) 
    {
      int i = write(m_socket, send_frame, sizeof(struct can_frame));
      printf("Written %d\n", i);
      return false;
    }
    else
      return true;
  }

  void VescCanInterface::requestFWVersion() 
  {
    can_frame frame;
    frame.can_id = genEId(m_dev_id, vesc_can_driver::CAN_PACKET_PROCESS_SHORT_BUFFER); 
    frame.can_dlc = 0x1;  
    frame.data[0] = host_id;
    int num_bytes = write(m_socket, &frame, sizeof(struct can_frame));
    if (num_bytes < 0)
    {
      printf("num bytes negative");
      perror("error writing");
    }
    
  }

  uint32_t VescCanInterface::genEId(uint32_t id, uint32_t  packet_id)
  {
    id |= packet_id << 8;  // Next lowest byte is the packet id.
    return(id |= 0x80000000);              // Send in Extended Frame Format.
  }

  void VescCanInterface::setDutyCycle(double duty_cycle) 
  {
    printf("Duty cycle data: %f\n", duty_cycle);
    can_frame frame;
    int32_t ind = 0;
    frame.can_id = genEId(m_dev_id, vesc_can_driver::CAN_PACKET_SET_DUTY); 
    bldc_buffer_append_float32(frame.data, duty_cycle, 1e5, &ind); 
    frame.can_dlc = 0x8; 
    int err = send(&frame);
    // if(err < 0)
    // {
    //   perror("Error sending DC");
    //   printf("send return: %d\n", err);
    // }
  }

  void VescCanInterface::setCurrent(double current) 
  {
    can_frame frame;
    int32_t ind = 0;
    frame.can_id = genEId(m_dev_id, vesc_can_driver::CAN_PACKET_SET_CURRENT); 
    bldc_buffer_append_float32(frame.data, current,1e3, &ind); 
    frame.can_dlc = 0x8;  
    frame.data[0] = 0x02;
    send(&frame);
  }
  void VescCanInterface::setBrake(double brake) 
  {
    can_frame frame;
    int32_t ind = 0;
    frame.can_id = genEId(m_dev_id, vesc_can_driver::CAN_PACKET_SET_CURRENT_BRAKE); 
    bldc_buffer_append_float32(frame.data, brake,1e3, &ind); 
    frame.can_dlc = 0x8;  
    send(&frame);
  }

  void VescCanInterface::setSpeed(double speed) 
  {
    can_frame frame;
    int32_t ind = 0;
    frame.can_id = genEId(m_dev_id, vesc_can_driver::CAN_PACKET_SET_RPM); 
    bldc_buffer_append_float32(frame.data, speed,1e0, &ind); 
    frame.can_dlc = 0x8;  
    int err = send(&frame);
    if(err < 0)
    {
      perror("Error sending RPM");
      printf("send return: %d\n", err);
    }
  }

  void VescCanInterface::setPosition(double position) 
  {
    can_frame frame;
    int32_t ind = 0;
    frame.can_id = genEId(m_dev_id, vesc_can_driver::CAN_PACKET_SET_POS); 
    bldc_buffer_append_float32(frame.data, position,1e6, &ind); 
    frame.can_dlc = 0x8;  
    send(&frame);
  }
 
  void VescCanInterface::start(const std::string& int_name, uint32_t dev_id) 
  {
    m_sock_err = 0;
    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(m_socket < 0)
    {
      perror("Socket error");
      printf("ERROR cannot create CAN socket: %d\n", m_socket);
      return;
    }
    
    strcpy(m_ifr.ifr_name, int_name.c_str());
   
    if (ioctl(m_socket, SIOCGIFINDEX, &m_ifr))
    {
      perror("IOCTL error");
      printf("ERROR unable to connect to interface: %s, error code:%d\n", m_ifr.ifr_name, m_sock_err);
      return;
    }
    m_addr.can_family = AF_CAN;
    m_addr.can_ifindex = m_ifr.ifr_ifindex;

    // struct timeval tv;
    // tv.tv_sec = 0;
    // tv.tv_usec = 100*1000;
    // printf("Setting Timeout\n");
    // if(setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) < 0) // Might be useful
    // {
    //   perror("Async set error");
    // }
    if (bind(m_socket, (struct sockaddr *)&m_addr, sizeof(m_addr)))
    {
      perror("Bind error");
      printf("ERROR unable to bind to interface: %s, error code:%d\n", m_ifr.ifr_name, m_sock_err);
      return;
    }
    can_err_mask_t optval = CAN_ERR_MASK;
    if(setsockopt(m_socket, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &optval, sizeof(optval)) < 0)
    {
      perror("sockopt error");
      return;
    }
    status_.connection_state = vesc_can_driver::WAITING_FOR_FW;
    m_dev_id = dev_id;
    host_id = 0x78; //TODO make it random
    // Set filter for CANID
    struct can_filter rfilter[2];
    rfilter[0].can_id = m_dev_id;
    rfilter[0].can_mask = 0xFF;
    rfilter[1].can_id = host_id;
    rfilter[1].can_mask = 0xFF;
    setsockopt(m_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    // start threads
    rx_thread_run_ = true;
    update_thread_run_ = true;
    pthread_create(&rx_thread_handle_, NULL, &VescCanInterface::rx_thread_helper, this);
    pthread_create(&update_thread_handle_, NULL, &VescCanInterface::update_thread_helper, this);
  }

  void VescCanInterface::stop() 
  {
    // stops the motor
    // setDutyCycle(0.0);

    // tell the io thread to stop
    rx_thread_run_ = false;
    update_thread_run_ = false;

    // wait for io thread to actually exit
    pthread_join(rx_thread_handle_, nullptr);
    pthread_join(update_thread_handle_, nullptr);
    if (close(m_socket) < 0) 
    {
      printf("Close");
    }
  }

  void VescCanInterface::get_status(VescStatusStruct *status) 
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    *status = status_;
  }

  void VescCanInterface::wait_for_status(VescStatusStruct *status) 
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    // wait for new data
    status_cv_.wait(lk);
    *status = status_;
  }
}

void bldc_buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
  bldc_buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void bldc_buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
  bldc_buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

int16_t bldc_buffer_get_int16(const uint8_t *buffer, int32_t *index) {
  int16_t res =   ((uint16_t) buffer[*index]) << 8 |
                  ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

uint16_t bldc_buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
  uint16_t res =  ((uint16_t) buffer[*index]) << 8 |
                  ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

int32_t bldc_buffer_get_int32(const uint8_t *buffer, int32_t *index) {
  int32_t res =   ((uint32_t) buffer[*index]) << 24 |
                  ((uint32_t) buffer[*index + 1]) << 16 |
                  ((uint32_t) buffer[*index + 2]) << 8 |
                  ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
}

uint32_t bldc_buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
  uint32_t res =  ((uint32_t) buffer[*index]) << 24 |
                  ((uint32_t) buffer[*index + 1]) << 16 |
                  ((uint32_t) buffer[*index + 2]) << 8 |
                  ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
}

float bldc_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
  return (float)bldc_buffer_get_int16(buffer, index) / scale;
}

float bldc_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
  return (float)bldc_buffer_get_int32(buffer, index) / scale;
}
