#ifndef MAINMENUPRESENTER_HPP
#define MAINMENUPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class MainMenuView;

class MainMenuPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    MainMenuPresenter(MainMenuView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~MainMenuPresenter() {}

private:
    MainMenuPresenter();

    void onVehicleDataUpdated(int32_t velocity,
                              uint8_t faultCode,
                              uint8_t reverse,
                              uint8_t mode,
                              uint8_t seinL,
                              uint8_t seinR,
                              uint8_t hazard,
                              uint8_t beam,
                              uint8_t batt,
                              uint8_t charge);

    MainMenuView& view;
};

#endif // MAINMENUPRESENTER_HPP
