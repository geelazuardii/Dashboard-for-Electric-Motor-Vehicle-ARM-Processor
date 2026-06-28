#include <gui/mainmenu_screen/MainMenuView.hpp>
#include <gui/mainmenu_screen/MainMenuPresenter.hpp>

MainMenuPresenter::MainMenuPresenter(MainMenuView& v)
    : view(v)
{

}

void MainMenuPresenter::onVehicleDataUpdated(int32_t velocity,
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
    view.updateVehicleData(velocity, faultCode, reverse, mode,
                           seinL, seinR, hazard, beam, batt, charge);
}

void MainMenuPresenter::activate()
{

}

void MainMenuPresenter::deactivate()
{

}
