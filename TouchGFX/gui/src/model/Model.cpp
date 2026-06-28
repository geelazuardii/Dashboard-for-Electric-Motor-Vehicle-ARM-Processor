#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include <cstdint>

extern "C" {
    extern volatile int32_t velocity;
    extern volatile uint8_t faultCode;
    extern volatile uint8_t reverse;
    extern volatile uint8_t fardriver_mode;

    extern volatile uint8_t seinL_val;
    extern volatile uint8_t seinR_val;
    extern volatile uint8_t hazard_val;
    extern volatile uint8_t beam_val;
    extern volatile uint8_t batt_val;
    extern volatile uint8_t charge_val;
}

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	static int32_t lastVelocity = -9999;
	    static uint8_t lastFault = 0xFF;
	    static uint8_t lastReverse = 0xFF;
	    static uint8_t lastMode = 0xFF;
	    static uint8_t lastSeinL = 0xFF;
	    static uint8_t lastSeinR = 0xFF;
	    static uint8_t lastHazard = 0xFF;
	    static uint8_t lastBeam = 0xFF;
	    static uint8_t lastBatt = 0xFF;
	    static uint8_t lastCharge = 0xFF;

	    if (velocity != lastVelocity ||
	        faultCode != lastFault ||
	        reverse != lastReverse ||
	        fardriver_mode != lastMode ||
	        seinL_val != lastSeinL ||
	        seinR_val != lastSeinR ||
	        hazard_val != lastHazard ||
	        beam_val != lastBeam ||
	        batt_val != lastBatt ||
	        charge_val != lastCharge)
	    {
	        lastVelocity = velocity;
	        lastFault = faultCode;
	        lastReverse = reverse;
	        lastMode = fardriver_mode;
	        lastSeinL = seinL_val;
	        lastSeinR = seinR_val;
	        lastHazard = hazard_val;
	        lastBeam = beam_val;
	        lastBatt = batt_val;
	        lastCharge = charge_val;

	        if (modelListener != 0)
	        {
	            modelListener->onVehicleDataUpdated(
	                velocity,
	                faultCode,
	                reverse,
	                fardriver_mode,
	                seinL_val,
	                seinR_val,
	                hazard_val,
	                beam_val,
	                batt_val,
	                charge_val
	            );
	        }
	    }
}
