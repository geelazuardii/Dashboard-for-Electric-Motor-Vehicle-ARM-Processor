#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP
#include <cstdint>

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    virtual void onVehicleDataUpdated(int32_t velocity,
                                      uint8_t faultCode,
                                      uint8_t reverse,
                                      uint8_t mode,
                                      uint8_t seinL,
                                      uint8_t seinR,
                                      uint8_t hazard,
                                      uint8_t beam,
                                      uint8_t batt,
                                      uint8_t charge) {}

    void bind(Model* m)
    {
        model = m;
    }
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
