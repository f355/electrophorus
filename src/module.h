#ifndef MODULE_H
#define MODULE_H

class Module {
 protected:
  ~Module() = default;

 public:
  virtual bool listens_to_rx();  // should return true if the module needs to run when data is received from LinuxCNC
  virtual bool is_base();        // should return true if the module needs to run in the "fast" base thread
  virtual bool is_servo();       // should return true if the module needs to run in the "slow" servo thread

  virtual void run_base();   // called on each tick of the base thread, if needed
  virtual void run_servo();  // called on each tick of the servo thread, if needed
  virtual void on_rx();      // called when data is received from LinuxCNC, if needed
};

#endif
