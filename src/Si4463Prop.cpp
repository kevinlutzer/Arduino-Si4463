#include "Si4463Prop.h"
#include "si4464_config.h"

Si4463Properties::Si4463Properties(uint16_t prop, uint8_t *buf, size_t len) {

  this->param_bytes = (uint8_t *)malloc(sizeof(uint8_t) * len);
  if (param_bytes == NULL) {
    memcpy(this->param_bytes, buf, len);
  }

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

void Si4463Properties::serializeHeader(uint8_t *buf) {
  buf[0] = SI4463_SET_PROPERTY_CMD;
  buf[1] = this->getProp() >> 8;   // GROUP
  buf[2] = this->getLen();         // NUM_PROPS
  buf[3] = this->getProp() & 0xff; // START_PROP
}

void Si4463Properties::serialize(uint8_t *buf) {
  this->serializeHeader(buf);
  memcpy(buf + SI4463_SET_PROPERTY_HEADER_LEN, this->getParams(), this->getLen() + SI4463_SET_PROPERTY_HEADER_LEN);
}