#ifndef SHOWVICTIMDIALOG_H
#define SHOWVICTIMDIALOG_H

#include <QDialog>

namespace Ui {
class ShowVictimDialog;
}

namespace graphics{

class ShowVictimDialog : public QDialog
{
    Q_OBJECT
    
signals:

    void signalVictimConfirmed(uint victimID);
    void signalNoVictimRetrieved(uint victimID);

public:
    explicit ShowVictimDialog(QWidget *parent = 0, uint id = 0, QList<QImage> imgs = QList<QImage>());
    ~ShowVictimDialog();

    uint getVictimID();

public slots:

    void nextFrame();
    void previousFrame();

    void onVictimAccepted();
    void onVictimRefused();

    void onPostponeDecision();

private:
    void showFrame();

    Ui::ShowVictimDialog *ui;
    uint victimID;
    QList<QImage> images;
    int actualFrame;
};

}
#endif // SHOWVICTIMDIALOG_H
