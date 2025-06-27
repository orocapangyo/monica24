#include "MonicaCommModule.h"

#define DEBUG_COMM_MODULExx

MonicaConfigComm* MonicaCommModule::config_comm_ = nullptr;
MonicaRosComm* MonicaCommModule::ros_comm_ = nullptr;
MonicaEEPROM* MonicaCommModule::eeprom_module_ = nullptr;    

MonicaCommModule::MonicaCommModule(MonicaConfigComm* config_comm, MonicaRosComm* ros_comm, MonicaEEPROM* eeprom_module)
{
    config_comm_ = config_comm;
    ros_comm_ = ros_comm;
    eeprom_module_ = eeprom_module;
}
    
MonicaCommModule::~MonicaCommModule()
{

}

void MonicaCommModule::initialize()
{
  
  if ((config_comm_ == nullptr)
   || (ros_comm_ == nullptr)
   || (eeprom_module_ == nullptr))
  {
    return;
  }

  eeprom_module_->initialize();

  digitalWrite(LED_BUILTIN, 1);

  config_comm_->initialize();
  config_comm_->wait_to_receive();

  digitalWrite(LED_BUILTIN, 0);



  if (eeprom_module_->is_enable_to_use())
  {
      ros_comm_->set_used_wifi(eeprom_module_->load_used_wifi());
      ros_comm_->set_domain_id(eeprom_module_->load_domain_id());
      ros_comm_->set_ssid(eeprom_module_->load_ssid());
      ros_comm_->set_password(eeprom_module_->load_password());
      ros_comm_->set_agent_ip(eeprom_module_->load_agent_ip());
      ros_comm_->set_port(eeprom_module_->load_port());
  }
  


  if (ros_comm_->is_used_wifi())
  {
    ros_comm_->use_wifi_transform();
  }
  else
  {
    ros_comm_->use_serial_transform();
  }

  delay(2000);

  ros_comm_->Initialize();
}



