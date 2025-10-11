#ifndef MACHINEDEF_TYPES_H
#define MACHINEDEF_TYPES_H

typedef struct {
  const char *name;
  uint8_t port;
  uint8_t pin;
  bool pull_down;
  bool invert;
} inputPin_t;

typedef struct {
  const char *name;
  uint8_t port;
  uint8_t pin;
  bool invert;
} outputPin_t;

#endif
