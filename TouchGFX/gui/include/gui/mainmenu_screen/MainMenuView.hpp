#ifndef MAINMENUVIEW_HPP
#define MAINMENUVIEW_HPP
#include <cstdint>

#include <gui_generated/mainmenu_screen/MainMenuViewBase.hpp>
#include <gui/mainmenu_screen/MainMenuPresenter.hpp>

class MainMenuView : public MainMenuViewBase
{
public:
    MainMenuView();
    virtual ~MainMenuView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();

    void updateVehicleData(int32_t velocity,
                               uint8_t faultCode,
                               uint8_t reverse,
                               uint8_t mode,
                               uint8_t seinL,
                               uint8_t seinR,
                               uint8_t hazard,
                               uint8_t beam,
                               uint8_t batt,
                               uint8_t charge);

        int32_t currentVelocity;
        uint8_t currentFault;
        uint8_t currentReverse;
        uint8_t currentMode;
        uint8_t currentSeinL;
        uint8_t currentSeinR;
        uint8_t currentHazard;
        uint8_t currentBeam;
        uint8_t currentBatt;
        uint8_t currentCharge;


private:
    static const uint16_t SPEED_BUFFER_SIZE = 6;
    touchgfx::Unicode::UnicodeChar speedBuffer[SPEED_BUFFER_SIZE];
    int32_t lastSpeed;

    uint32_t tickCounter;
        uint32_t sec;


    void setPair(touchgfx::Image& onObj, touchgfx::Image& offObj, bool on);
    void updateDashboard();




protected:
//    int currentSpeed;
//    bool isIncreasing;
};

#endif // MAINMENUVIEW_HPP
