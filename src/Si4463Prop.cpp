#include "Si4463Prop.h"

Si4463Properties::Si4463Properties(uint16_t prop, uint8_t *param_bytes,
                                   size_t len) {
  this->param_bytes = (uint8_t *)malloc(sizeof(uint8_t) * len);
  memcpy(this->param_bytes, param_bytes, len);

  this->prop = prop;
  this->len = len;
}

uint16_t Si4463Properties::getProp() { return prop; }
uint8_t *Si4463Properties::getParams() { return param_bytes; }
size_t Si4463Properties::getLen() { return len; }

Si4463Properties::~Si4463Properties() { free(this->param_bytes); }

void Si4463Properties::setByte(size_t index, uint8_t value) {
  if (index < len) {
    this->param_bytes[index] = value;
  }
}