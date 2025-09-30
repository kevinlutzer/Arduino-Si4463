#ifndef SI4463_PROP_H
#define SI4463_PROP_H

#include <Arduino.h>

class Si4463Property {

public:
  /**
   * @brief Constructs a Si4463Property object. Because this constructor
   * will allocate memory for the param_bytes, it **must** be deconstructed when
   * no longer needed
   * @param prop The property to set
   * @param param_bytes Pointer to the parameters to set. This memory will be
   * copied
   * @param len Length of the param_bytes array
   */
  Si4463Property(uint16_t prop, uint8_t *param_bytes, size_t len);
  ~Si4463Property();

  uint16_t getProp();
  uint8_t *getParams();
  size_t getLen();

  void setByte(size_t index, uint8_t value);

private:
  uint16_t prop;
  uint8_t *param_bytes;
  size_t len;
};

#endif // SI4463_PROP_H