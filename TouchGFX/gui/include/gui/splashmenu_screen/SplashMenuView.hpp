#ifndef SPLASHMENUVIEW_HPP
#define SPLASHMENUVIEW_HPP

#include <gui_generated/splashmenu_screen/SplashMenuViewBase.hpp>
#include <gui/splashmenu_screen/SplashMenuPresenter.hpp>

class SplashMenuView : public SplashMenuViewBase
{
public:
    SplashMenuView();
    virtual ~SplashMenuView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SPLASHMENUVIEW_HPP
