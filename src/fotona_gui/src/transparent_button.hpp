#pragma once

#include <QPushButton>
#include <QWidget>
#include <QObject>
#include <QGraphicsOpacityEffect>

class TransparentButton : public QPushButton {
    // Q_OBJECT
public:
    TransparentButton(char const* label, QWidget* parent) : QPushButton(label, parent) {}
protected:
    void enterEvent(QEvent* const event) override {
        const_cast<QWidget*>(dynamic_cast<QWidget const*>(this->parent()))->update();
        puts("entered");
    }
    void leaveEvent(QEvent* const event) override {
        const_cast<QWidget*>(dynamic_cast<QWidget const*>(this->parent()))->update();
        puts("left");
    }
    void focusInEvent(QFocusEvent* const event) override {
        // Change the background color when the button is focused
        this->QPushButton::focusInEvent(event);
        this->QPushButton::update();
        this->QPushButton::graphicsEffect()->update();
    }

    void focusOutEvent(QFocusEvent* const event) override {
        // Revert to the original background color when the button loses focus
        this->QPushButton::setStyleSheet("");
        this->QPushButton::focusOutEvent(event);
        this->QPushButton::graphicsEffect()->update();
    }
};
