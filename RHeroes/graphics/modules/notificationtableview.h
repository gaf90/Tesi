#ifndef NOTIFICATIONTABLEVIEW_H
#define NOTIFICATIONTABLEVIEW_H

#include <QTableView>

namespace graphics{


class NotificationTableView: public QTableView
{
    Q_OBJECT
public:
    NotificationTableView();

signals:
    void signalFilteringRequested(QModelIndex index);
    void signalUnfilteringRequested();
    void signalArchived(QModelIndex index);
    void signalDeleted(QModelIndex index);

private slots:
    void contextMenuEvent(QContextMenuEvent *event);
    void onFiltering();
    void onUnfiltering();
//    void onClick(QModelIndex index);

private:
    QMenu *menu;
    QAction *filterAction;
    QAction *unfilterAction;
    QModelIndex lastIndex;
};

}
#endif // NOTIFICATIONTABLEVIEW_H
