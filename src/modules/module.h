#ifndef MODULE_H
#define MODULE_H

class Module {
 protected:
  ~Module() = default;

 public:
  virtual bool is_stepgen() { return false; }  // should return true if the module generates steps in realtime
  virtual void make_steps() {}  // called on each tick of the stepgen ticker, if is_stepgen() returns true
  virtual void on_rx() {}       // called every time data is received from LinuxCNC
};

#endif
