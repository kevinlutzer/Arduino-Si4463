#include "Si4463Prop.h"
#include "si4464_config.h"

Si4463Command::Si4463Command(uint8_t cmd, uint8_t *buf, size_t len) {
  this->param_bytes = new uint8_t[len];
  memcpy(this->param_bytes, buf, len);

  this->cmd = cmd;
  this->len = len;
}

uint16_t Si4463Command::getCmd() { return this->cmd; }

Si4463Command::~Si4463Command() { delete[] this->param_bytes; }

void Si4463Command::setByte(size_t index, uint8_t value) {
  if (index < this->len) {
    this->param_bytes[index] = value;
  }
}

uint8_t *Si4463Command::getParams() { return this->param_bytes; }
size_t Si4463Command::getLen() { return this->len; }

Si4463Property::Si4463Property(uint16_t prop, uint8_t *param_bytes, size_t len)
    : Si4463Command(SI4463_SET_PROPERTY_CMD, param_bytes, len) // Calls base constructor
{
    this->prop = prop;
}
 
Si4463Property::~Si4463Property() {
    Si4463Command::~Si4463Command();
}  