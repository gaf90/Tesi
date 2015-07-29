#ifndef POPUPWIDGET_H
#define POPUPWIDGET_H

#define BS_ACCEPTED_MESSAGE "MessageAcceptedAndGood"
#define BS_REFUSED_MESSAGE "MessageRefusedAndBad"

#include <QWidget>
#include <QString>

namespace Ui {
    class PopupWidget;
}

namespace graphics{

class PopupWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PopupWidget(QWidget *parent = 0, int id = -1, QString module = "", QString message = "");
    virtual ~PopupWidget();

    int getId();

signals:
    void signalOperatorBehaviour(int id, QString module, QString result);

private slots:
    void onCancel();
    void onOk();

private:
    Ui::PopupWidget *ui;

    QString module;
    int identifier;
};

}
#endif // POPUPWIDGET_H
