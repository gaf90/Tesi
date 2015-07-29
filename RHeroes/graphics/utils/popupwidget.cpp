#include "popupwidget.h"
#include "ui_popupwidget.h"

namespace graphics{

PopupWidget::PopupWidget(QWidget *parent, int id, QString module, QString message) :
    QWidget(parent),
    ui(new Ui::PopupWidget), module(module), identifier(id)
{
    ui->setupUi(this);
    ui->messageLabel->setText(message);
    this->setAttribute(Qt::WA_TranslucentBackground);
    connect(ui->okButton, SIGNAL(clicked()), this, SLOT(onOk()));
    connect(ui->noGoodButton, SIGNAL(clicked()), this, SLOT(onCancel()));
}

PopupWidget::~PopupWidget()
{
    delete ui;
}

void PopupWidget::onCancel()
{
    emit signalOperatorBehaviour(identifier, module, BS_ACCEPTED_MESSAGE);
//    this->close();
}

void PopupWidget::onOk()
{
    emit signalOperatorBehaviour(identifier, module, BS_REFUSED_MESSAGE);
//    this->close();
}

int PopupWidget::getId()
{
    return this->identifier;
}

}

