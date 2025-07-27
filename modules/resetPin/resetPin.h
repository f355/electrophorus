#ifndef RESETPIN_H
#define RESETPIN_H

#include "modules/module.h"

Module* createResetPin(JsonObject module, RemoraComms* comms);

class ResetPin : public Module
{
    private:

        volatile bool *ptrReset; // pointer to the data source

        Pin *pin;

    public:

        ResetPin(volatile bool&, std::string);
        virtual void update(void);
};

#endif
