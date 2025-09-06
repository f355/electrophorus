#ifndef MODULE_H
#define MODULE_H

class Module {
 protected:
  ~Module() = default;

 public:
  virtual bool is_servo();
  virtual bool is_base();
  virtual void run_servo();
  virtual void run_base();
};

#endif
