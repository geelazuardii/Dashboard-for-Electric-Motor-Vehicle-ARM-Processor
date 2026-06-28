#ifndef SPLASHMENUPRESENTER_HPP
#define SPLASHMENUPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SplashMenuView;

class SplashMenuPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SplashMenuPresenter(SplashMenuView& v);

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

    virtual ~SplashMenuPresenter() {}

private:
    SplashMenuPresenter();

    SplashMenuView& view;
};

#endif // SPLASHMENUPRESENTER_HPP
