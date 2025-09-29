#ifndef SI4463_PROP_H
#define SI4463_PROP_H

#include <Arduino.h>

class Si4463Properties {

public:
  /**
   * @brief Constructs a Si4463Properties object. Because this constructor
   * will allocate memory for the param_bytes, it **must** be deconstructed when
   * no longer needed
   * @param prop The property to set
   * @param param_bytes Pointer to the parameters to set. This memory will be
   * copied
   * @param len Length of the param_bytes array
   */
  Si4463Properties(uint16_t prop, uint8_t *param_bytes, size_t len);
  ~Si4463Properties();

  uint16_t getProp();
  uint8_t *getParams();
  size_t getLen();

  /**
   * @brief Sets a single byte in the parameters array
   * @param index Index of the byte to set
   * @param value Value to set the byte to
   */
  void setByte(size_t index, uint8_t value);

  /**
   * @brief Serializes the property into a buffer that can be sent to the Si4463
   * @param buf Buffer to serialize into. This must be at least
   * SI4463_SET_PROPERTY_HEADER_LEN + getLen() bytes long
   */
  void serialize(uint8_t *buf);

  void serializeHeader(uint8_t *buf);

private:
  uint16_t prop;
  uint8_t *param_bytes;
  size_t len;
};

#endif // SI4463_PROP_H