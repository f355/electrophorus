#ifndef MODULE_H
#define MODULE_H

class Module {
 protected:
  ~Module() = default;

 public:
  virtual bool listens_to_rx();  // return true if the module should run when data is received from LinuxCNC
  virtual bool is_stepgen();     // return true if the module should run in the stepgen ticker

  virtual void make_steps();  // called on each tick of the stepgen ticker, if needed
  virtual void on_rx();       // called when data is received from LinuxCNC, if needed
};

#endif
