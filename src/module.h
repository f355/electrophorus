#ifndef MODULE_H
#define MODULE_H

class Module {
 protected:
  ~Module() = default;

 public:
  virtual bool is_stepgen();  // should return true if the module is a stepgen and should be ticked

  virtual void make_steps();  // called on each tick of the base stepgen ticker, if is_stepgen() returns true
  virtual void on_rx();       // called when data is received from LinuxCNC
};

#endif
