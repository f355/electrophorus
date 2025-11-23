#pragma once

class Module {
 protected:
  ~Module() = default;

 public:
  virtual bool IsStepgen() { return false; }  // should return true if the module generates steps in realtime
  virtual void MakeSteps() {}                 // called on each tick of the stepgen ticker, if is_stepgen() returns true
  virtual void OnRx() {}                      // called every time data is received from LinuxCNC
};
