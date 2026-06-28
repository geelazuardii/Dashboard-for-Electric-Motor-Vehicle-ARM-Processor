#include <gui/mainmenu_screen/MainMenuView.hpp>

extern "C" {
	#include "bms_jk.h"

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
    extern volatile uint8_t jk_bms_soc;
    extern volatile float jk_bms_current;
    extern volatile uint8_t jk_bms_connected;
}

MainMenuView::MainMenuView()
    : currentVelocity(0),
      currentFault(0),
      currentReverse(0),
      currentMode(0),
      currentSeinL(0),
      currentSeinR(0),
      currentHazard(0),
      currentBeam(0),
      currentBatt(0),
      currentCharge(0),
      lastSpeed(-1),
	  tickCounter(0),
	  sec(0)

{
}

void MainMenuView::setupScreen()
{
    MainMenuViewBase::setupScreen();

    txtSpeed.setWildcard(speedBuffer);
    txtSoc.setWildcard(txtSocBuffer);

    currentVelocity = velocity;
    currentFault    = faultCode;
    currentReverse  = reverse;
    currentMode     = fardriver_mode;
    currentSeinL    = seinL_val;
    currentSeinR    = seinR_val;
    currentHazard   = hazard_val;
    currentBeam     = beam_val;
    currentBatt     = batt_val;
    currentCharge   = charge_val;

    updateDashboard();
}

void MainMenuView::tearDownScreen()
{
    MainMenuViewBase::tearDownScreen();
}

void MainMenuView::handleTickEvent()
{
    MainMenuViewBase::handleTickEvent();

    currentVelocity = velocity;
    currentFault    = faultCode;
    currentReverse  = reverse;
    currentMode     = fardriver_mode;
    currentSeinL    = seinL_val;
    currentSeinR    = seinR_val;
    currentHazard   = hazard_val;
    currentBeam     = beam_val;
    currentBatt     = batt_val;
    currentCharge   = charge_val;







    //counter

    // asumsi 60 tick = 1 detik
    // tick jalan terus
    /*tickCounter++;

    // blink lebih lambat
    bool blinkState = ((tickCounter / 60) % 2);

    // 120 tick = 1 detik
    if (tickCounter >= 120)
    {
        tickCounter = 0;
        sec++;

        // looping ulang
        if (sec >= 35)
        {
            sec = 0;
        }
    }

    // reset semua
    seinL_val = 0;
    seinR_val = 0;
    hazard_val = 0;
    beam_val = 0;

    // =======================
    // STATE MACHINE
    // =======================

    // 8 - 13 detik -> sein kiri
    if (sec >= 8 && sec < 13)
    {
        seinL_val = blinkState;
    }

    // 13 - 18 detik -> sein kanan
    else if (sec >= 13 && sec < 18)
    {
        seinR_val = blinkState;
    }

    // 18 - 23 detik -> hazard
    else if (sec >= 18 && sec < 23)
    {
        hazard_val = blinkState;
    }

    // 23 - 28 detik -> beam nyala terus
    else if (sec >= 23 && sec < 28)
    {
        beam_val = 1;
    }*/

    // 28 - 35 -> semua mati









    updateDashboard();
}

void MainMenuView::setPair(touchgfx::Image& onObj, touchgfx::Image& offObj, bool on)
{
    if (onObj.isVisible() != on)
    {
        onObj.setVisible(on);
        onObj.invalidate();
    }

    if (offObj.isVisible() == on)
    {
        offObj.setVisible(!on);
        offObj.invalidate();
    }
}

void MainMenuView::updateVehicleData(int32_t velocity,
                                     uint8_t faultCode,
                                     uint8_t reverse,
                                     uint8_t mode,
                                     uint8_t seinL,
                                     uint8_t seinR,
                                     uint8_t hazard,
                                     uint8_t beam,
                                     uint8_t batt,
                                     uint8_t charge)
{
    currentVelocity = velocity;
    currentFault = faultCode;
    currentReverse = reverse;
    currentMode = mode;
    currentSeinL = seinL;
    currentSeinR = seinR;
    currentHazard = hazard;
    currentBeam = beam;
    currentBatt = batt;
    currentCharge = charge;

    updateDashboard();
}

//void MainMenuView::updateDashboard()
//{
//    if (lastSpeed != currentVelocity)
//    {
//        lastSpeed = currentVelocity;
//        touchgfx::Unicode::snprintf(speedBuffer, SPEED_BUFFER_SIZE, "%d", currentVelocity);
//        txtSpeed.invalidate();
//    }
//
//    setPair(error,  error_off,  currentFault != 0U);
//    setPair(r,      r_off,      currentReverse != 0U);
//
//    setPair(beam,   beam_off,   currentBeam   != 0U);
//    setPair(batt,   batt_off,   currentBatt   != 0U);
//    setPair(charge, charge_off, currentCharge != 0U);
//
//    setPair(hazard, hazard_0ff, currentHazard != 0U);
//
//    setPair(seinl, seinl_off, (currentSeinL != 0U) && (currentHazard == 0U));
//    setPair(seinr, seinr_off, (currentSeinR != 0U) && (currentHazard == 0U));
//
//    setPair(mode1, mode1_off, (currentMode == 1U));
//    setPair(mode2, mode2_off, (currentMode == 2U));
//    setPair(mode3, mode3_off, (currentMode == 3U));
//}

void MainMenuView::updateDashboard()
{
    // speed
    if (lastSpeed != velocity)
    {
        lastSpeed = velocity;
        touchgfx::Unicode::snprintf(speedBuffer, SPEED_BUFFER_SIZE, "%d", velocity);
        txtSpeed.invalidate();
    }

    //bms
    touchgfx::Unicode::snprintf(
    txtSocBuffer,
    TXTSOC_SIZE,
    "d%%",
    (unsigned int)jk_bms_soc
    );
    txtSoc.invalidate();

     //icon on/off
    setPair(error,  error_off,  faultCode != 0U);
    setPair(r,      r_off,      reverse != 0U);

    setPair(beam,   beam_off,   beam_val   != 0U);
    setPair(batt, batt_off, (jk_bms_connected != 0U) && (jk_bms_soc <= 20U));
    setPair(charge, charge_off, (jk_bms_connected != 0U) && (jk_bms_current > 0.2f));

    setPair(hazard, hazard_0ff, hazard_val != 0U);

    // sein mati kalau hazard aktif
    setPair(seinl, seinl_off, (seinL_val != 0U) && (hazard_val == 0U));
    setPair(
    		seinr, seinr_off, (seinR_val != 0U) && (hazard_val == 0U));

    // mode
    setPair(mode1, mode1_off, (fardriver_mode == 1U));
    setPair(mode2, mode2_off, (fardriver_mode == 2U));
    setPair(mode3, mode3_off, (fardriver_mode == 3U));
}

