#ifndef SHOWVICTIMWIDGET_H
#define SHOWVICTIMWIDGET_H

#include <QWidget>

namespace Ui {
class ShowVictimWidget;
}

namespace graphics{

class ShowVictimWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit ShowVictimWidget(QWidget *parent = 0);
    ~ShowVictimWidget();

    void setImages(const QList<QImage> &imgs);

private slots:

    void nextFrame();
    void previousFrame();


    
private:
    void showFrame();

    Ui::ShowVictimWidget *ui;

    QList<QImage> images;
    int actualFrame;
};

}
#endif // SHOWVICTIMWIDGET_H
