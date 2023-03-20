#pragma once

#include <QPushButton>
#include <QWidget>
#include <QObject>
#include <QGraphicsOpacityEffect>

#include <QMainWindow>

class TransparentButton : public QPushButton {
private:
    // Q_OBJECT
public:
    TransparentButton(char const* label, QWidget* parent=nullptr) : QPushButton(label, parent) {
        // this->setAttribute(Qt::WA_TransparentForMouseEvents);  // not relevant or helpful
        // this->setAttribute(Qt::WA_NoSystemBackground, false);  // use system background, no custom bg is provided
        // this->setAttribute(Qt::WA_OpaquePaintEvent, false);    // disable optimizations for opaque widgets
        // this->setAttribute(Qt::WA_PaintOnScreen, false);       // breaks the UI, don't enable
        // this->setAttribute(Qt::WA_TranslucentBackground, false);
        // this->setAttribute(Qt::WA_AlwaysStackOnTop, true);
    }
protected:
    void enterEvent(QEvent* const event) override {
    }
    void leaveEvent(QEvent* const event) override {
    }
    void focusInEvent(QFocusEvent* const event) override {
    }

    void focusOutEvent(QFocusEvent* const event) override {
    }
};
