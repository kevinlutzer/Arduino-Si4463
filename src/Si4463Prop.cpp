#include "Si4463Prop.h"

Si4463Property::Si4463Property(uint16_t prop, uint8_t *param_bytes,
                               size_t len) {
  this->param_bytes = (uint8_t *)malloc(sizeof(uint8_t) * len);
  memcpy(this->param_bytes, param_bytes, len);

  this->prop = prop;
  this->len = len;
}

uint16_t Si4463Property::getProp() { return prop; }
uint8_t *Si4463Property::getParams() { return param_bytes; }
size_t Si4463Property::getLen() { return len; }

Si4463Property::~Si4463Property() { free(this->param_bytes); }

void Si4463Property::setByte(size_t index, uint8_t value) {
  if (index < len) {
    this->param_bytes[index] = value;
  }
}