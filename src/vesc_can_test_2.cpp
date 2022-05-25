#include "vesc_can_interface.h" 
#include <iostream>

int main()
{
  std::cout << "Started demo, creating instace" << std::endl;
  vesc_can_driver::VescCanInterface vesc_driver_instance(0x02);
  std::cout << "Instance created, starting loop" << std::endl;
  for(int i=0; i<10; i++)
  {
    std::cout << "Loop num " << i << std::endl;
    std::cout << "got packet: " << vesc_driver_instance.requestState() << std::endl;
    std::cout << vesc_driver_instance.status_.voltage_input << std::endl;
  }
  std::cin.get();
}
