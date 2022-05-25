#include "vesc_can_interface.h" 
#include <iostream>

int main(int argc, char *argv[])
{
  int id = atoi(argv[1]);
  std::cout << "Started demo, creating instace" << std::endl;
  vesc_can_driver::VescCanInterface vesc_driver_instance(id);
  std::cout << "Instance created, starting loop" << std::endl;
  while(vesc_driver_instance.requestState()!=vesc_can_driver::CAN_PACKET_PROCESS_RX_BUFFER)
  {
      asm ("nop");
  }
  printf("FW version: %d.%d\n", vesc_driver_instance.status_.fw_version_major, 
         vesc_driver_instance.status_.fw_version_minor);
  printf("HW_name: %s\n", vesc_driver_instance.status_.hw_name);
  printf("UUID: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
         vesc_driver_instance.status_.uuid[0], vesc_driver_instance.status_.uuid[1], 
         vesc_driver_instance.status_.uuid[2], vesc_driver_instance.status_.uuid[3], 
         vesc_driver_instance.status_.uuid[4], vesc_driver_instance.status_.uuid[5], 
         vesc_driver_instance.status_.uuid[6], vesc_driver_instance.status_.uuid[7], 
         vesc_driver_instance.status_.uuid[8], vesc_driver_instance.status_.uuid[9], 
         vesc_driver_instance.status_.uuid[10], vesc_driver_instance.status_.uuid[11]);
  printf("Paring done: %d", vesc_driver_instance.status_.pairing_done);
  std::cin.get();
}
