#include "notificationtableview.h"
#include <QContextMenuEvent>
#include <QMenu>

namespace graphics{

NotificationTableView::NotificationTableView():
    lastIndex(QModelIndex())
{
    filterAction = new QAction(tr("Filter"), this);
    unfilterAction = new QAction(tr("remove filters"),this);
    menu = new QMenu(this);
    menu->addAction(filterAction);
    menu->addAction(unfilterAction);
    connect(filterAction, SIGNAL(triggered()), this, SLOT(onFiltering()));
    connect(unfilterAction, SIGNAL(triggered()), this, SLOT(onUnfiltering()));
//    connect(this, SIGNAL(clicked(QModelIndex)), this, SLOT(onClick(QModelIndex)));
}

void NotificationTableView::contextMenuEvent(QContextMenuEvent *event)
{
    lastIndex = this->indexAt(event->pos());
    menu->popup(mapToGlobal(event->pos()));
}

void NotificationTableView::onUnfiltering()
{
    signalUnfilteringRequested();
}

//void NotificationTableView::onClick(QModelIndex index)
//{
//}

void NotificationTableView::onFiltering()
{
    signalFilteringRequested(lastIndex);
}

}
