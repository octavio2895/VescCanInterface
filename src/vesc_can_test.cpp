#include "vesc_can_interface.h" 
#include <iostream>

int main(int argc, char *argv[])
{
  int id = atoi(argv[1]);
  std::cout << "Started demo, creating instace" << std::endl;
  vesc_can_driver::VescCanInterface vesc_dev;
  std::cout << "Init thread" << std::endl;
  vesc_dev.start(id);
  std::cout << "Instance created, starting loop" << std::endl;
  while(vesc_dev.m_pack != CAN_PACKET_PROCESS_RX_BUFFER)
  {
      asm ("nop");
  }
  printf("FW version: %d.%d\n", vesc_dev.status_.fw_version_major, 
         vesc_dev.status_.fw_version_minor);
  printf("HW_name: %s\n", vesc_dev.status_.hw_name);
  printf("UUID: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
         vesc_dev.status_.uuid[0], vesc_dev.status_.uuid[1], 
         vesc_dev.status_.uuid[2], vesc_dev.status_.uuid[3], 
         vesc_dev.status_.uuid[4], vesc_dev.status_.uuid[5], 
         vesc_dev.status_.uuid[6], vesc_dev.status_.uuid[7], 
         vesc_dev.status_.uuid[8], vesc_dev.status_.uuid[9], 
         vesc_dev.status_.uuid[10], vesc_dev.status_.uuid[11]);
  printf("Paring done: %d", vesc_dev.status_.pairing_done);
  std::cin.get();
}
