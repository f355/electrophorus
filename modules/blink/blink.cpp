#include "blink.h"

Module* createBlink(JsonObject module, PRUThread* thread)
{
    const char* pin = module["pin"];
    int frequency = module["frequency"];
        
    return new Blink(pin, thread->frequency, frequency);
}

Blink::Blink(std::string portAndPin, uint32_t threadFreq, uint32_t freq)
{
    this->periodCount = threadFreq / freq;
    this->blinkCount = 0;
    this->bState = false;

    this->blinkPin = (new Pin(portAndPin))->as_output();
    this->blinkPin->set(bState);
}

void Blink::update(void)
{
    ++this->blinkCount;
    if (this->blinkCount >= this->periodCount / 2)
    {
        this->blinkPin->set(this->bState=!this->bState);
        this->blinkCount = 0;
    }
}

void Blink::slowUpdate(void)
{
    return;
}
