#include "vesc_can_interface.h"
#include <cstring>
#include <iostream>

namespace vesc_can_driver
{
  VescCanInterface::VescCanInterface(int dev_id)
  {
    m_dev_id = dev_id;
    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(m_ifr.ifr_name, "can0");
    if (ioctl(m_socket, SIOCGIFINDEX, &m_ifr) < 0)
    {
      printf("ERROR unable to connect to interface: %s\n", m_ifr.ifr_name);
      return;
    }
    m_addr.can_family = AF_CAN;
    m_addr.can_ifindex = m_ifr.ifr_ifindex;
    
    bind(m_socket, (struct sockaddr *)&m_addr, sizeof(m_addr));
    
    struct can_filter rfilter[1];
    rfilter[0].can_id = m_dev_id;
    rfilter[0].can_mask = 0xFF;
    setsockopt(m_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
  }

  int VescCanInterface::requestState()
  {
    m_nbytes = read(m_socket, &m_recvframe, sizeof(struct can_frame));
    int ind = 0;
    uint8_t recv_id = m_recvframe.can_id;
    uint8_t pack_id = m_recvframe.can_id >> 8;  

    // Verify the frame id and can id:
    // if ( (status->id >0) && (recv_id != status->id))
      // return -1;

    //printf ("can id:%i, frame id:%i\n", recv_id, pack_id);

    // Read the selected data:
  // (1652470486.398796) can0 0000082D#020000
  // (1652470486.399054) can0 00000502#000005023630003D
  // (1652470486.399357) can0 00000502#070034000B503152
  // (1652470486.399629) can0 00000502#0E39363420000000
  // (1652470486.399842) can0 00000502#1500
  // (1652470486.400147) can0 00000702#2D01001651B9
    switch (pack_id)
    {
      case CAN_PACKET_FILL_RX_BUFFER:
        memcpy(m_rx_buffer + m_recvframe.data[0], m_recvframe.data + 1, 7);
        break;
      case CAN_PACKET_FILL_RX_BUFFER_LONG:
        break;
      case CAN_PACKET_PROCESS_RX_BUFFER:
        m_rx_last_buffer_id = m_recvframe.data[0];
        m_commands_send = m_recvframe.data[1];
        m_rxbuf_len = (uint16_t)m_recvframe.data[2]<<8 | (uint16_t)m_recvframe.data[3];
        m_crc_check = (uint16_t)m_recvframe.data[4]<<8 | (uint16_t)m_recvframe.data[5];
        if (crc16((unsigned char *)m_rx_buffer, m_rxbuf_len) == m_crc_check)
          process_rx_buffer();
        return CAN_PACKET_PROCESS_RX_BUFFER;
      case CAN_PACKET_PROCESS_SHORT_BUFFER:
        break;
      case CAN_PACKET_STATUS:
        status_.speed_erpm = (double)bldc_buffer_get_int32(m_recvframe.data, &ind);
        status_.current_motor = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) / 10.0;
        status_.duty_cycle = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) / 1000.0;
        break;
      
      case CAN_PACKET_STATUS_2:
        status_.charge_drawn = (double)bldc_buffer_get_int32(m_recvframe.data, &ind) /1000.0;
        status_.charge_regen = (double)bldc_buffer_get_int32(m_recvframe.data, &ind) /1000.0;
        break;

      case CAN_PACKET_STATUS_3:
        status_.energy_drawn = (double)bldc_buffer_get_int32(m_recvframe.data, &ind) /1000.0;
        status_.energy_regen = (double)bldc_buffer_get_int32(m_recvframe.data, &ind) /1000.0;
        break;
      
      case CAN_PACKET_STATUS_4:
        status_.temperature_pcb = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /10.0;
        status_.temperature_motor = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /10.0;
        status_.current_input = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /10.0;
        status_.current_pid_position = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /50.0;
        break;
      
      case CAN_PACKET_STATUS_5:
        status_.tacho = (double)bldc_buffer_get_int32(m_recvframe.data, &ind);
        status_.voltage_input = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /10.0;
        status_.reserved = bldc_buffer_get_int16(m_recvframe.data, &ind);
        break;

      case CAN_PACKET_STATUS_6:
        status_.ext_adc1 = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /100.0;
        status_.ext_adc2 = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /100.0;
        status_.ext_adc3 = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /100.0;
        status_.servo = (double)bldc_buffer_get_int16(m_recvframe.data, &ind) /100.0;
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
      case COMM_FW_VERSION:
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
        
        break;
    }

    return 0;
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
