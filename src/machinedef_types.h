#pragma once

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
