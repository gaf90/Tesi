#ifndef OBJECTTABLE_H
#define OBJECTTABLE_H

#include <QHash>

namespace Data{
    template<class T1, class T2, class T3>
    class ObjectTable
    {
    public:
        ObjectTable();
        ~ObjectTable();
        void insertRow(T1 rowElem, QHash<T2,T3> columnValues);
        void insertColumn(T2 columnElem, QHash<T1,T3> rowValues);
        void deleteRow(T1 row);
        void deleteColumn(T2 column);
        void multiplyForFactorRow(T1 row, double factor);
        void multiplyForFactorColumn(T2 column, double factor);
        T3 getValue(T1 row, T2 column);
        QList<T3> getRow(T1 row);
        QList<T3> getColumn(T2 column);
        QHash<T1,T3> getRowValue(T2 column);
        QHash<T2,T3> getColumnValue(T1 row);
        void clear();
        int rowCount();
        int columnCount();
        bool containsRow(T1 row);
        QList<T1> keys();

        inline QHash<T2,T3> operator[](T1 key)
        {
            return objectTable[key];
        }

    private:
        QHash<T1, QHash<T2,T3> > objectTable;

    };

    template<class T1, class T2, class T3>
    ObjectTable<T1,T2,T3>::ObjectTable()
    {
    }

    template<class T1, class T2, class T3>
    ObjectTable<T1,T2,T3>::~ObjectTable()
    {
    }

    template<class T1, class T2, class T3>
    void ObjectTable<T1,T2,T3>::insertRow(T1 rowElem, QHash<T2, T3> columnValues)
    {
        objectTable.insert(rowElem, columnValues);
    }

    template<class T1, class T2, class T3>
    void ObjectTable<T1,T2,T3>::insertColumn(T2 columnElem, QHash<T1, T3> rowValues)
    {
        T1 row;
        foreach(row, rowValues.keys()){
            objectTable[row].insert(columnElem, rowValues[row]);
        }
    }

    template<class T1, class T2, class T3>
    void ObjectTable<T1,T2,T3>::deleteRow(T1 row)
    {
        objectTable.remove(row);
    }

    template<class T1, class T2, class T3>
    void ObjectTable<T1,T2,T3>::deleteColumn(T2 column)
    {
        T1 row;
        foreach(row, objectTable.keys()){
            objectTable[row].remove(column);
        }
    }

    template<class T1, class T2, class T3>
    void ObjectTable<T1,T2,T3>::multiplyForFactorRow(T1 row, double factor)
    {
        T2 column;
        foreach(column, objectTable[row].keys()){
            objectTable[row][column] = factor * getValue(row, column);
        }
    }

    template<class T1, class T2, class T3>
    void ObjectTable<T1,T2,T3>::multiplyForFactorColumn(T2 column, double factor)
    {
        T1 row;
        foreach(row, objectTable.keys()){
            objectTable[row][column] = factor * getValue(row, column);
        }
    }

    template<class T1, class T2, class T3>
    T3 ObjectTable<T1,T2,T3>::getValue(T1 row, T2 column)
    {
        return objectTable[row][column];
    }

    template<class T1, class T2, class T3>
    QList<T3> ObjectTable<T1,T2,T3>::getRow(T1 row)
    {
        QList<T3> rowValues;
        T2 column;
        foreach(column, objectTable[row].keys()){
            rowValues.insert(objectTable[row][column]);
        }
    }

    template<class T1, class T2, class T3>
    QList<T3> ObjectTable<T1,T2,T3>::getColumn(T2 column)
    {
        QList<T3> columnValues;
        T1 row;
        foreach(row, objectTable.keys()){
            columnValues.insert(objectTable[row][column]);
        }
    }    

    template<class T1, class T2, class T3>
    QHash<T1, T3> ObjectTable<T1,T2,T3>::getRowValue(T2 column)
    {
        QHash<T1, T3> rowValueHash;
        foreach(T1 tempT1, objectTable.keys()){
            rowValueHash.insert(tempT1, objectTable[tempT1][column]);
        }
        return rowValueHash;
    }

    template<class T1, class T2, class T3>
    QHash<T2, T3> ObjectTable<T1,T2,T3>::getColumnValue(T1 row)
    {
        return objectTable[row];
    }

    template<class T1, class T2, class T3>
    void ObjectTable<T1,T2,T3>::clear()
    {
        objectTable.clear();
    }

    template<class T1, class T2, class T3>
    int ObjectTable<T1,T2,T3>::rowCount()
    {
        return objectTable.count();
    }

    template<class T1, class T2, class T3>
    int ObjectTable<T1,T2,T3>::columnCount()
    {
        if (objectTable.count() == 0)
            return 0;
        //assuming all rows has same number of column
        return objectTable[objectTable.keys().first()].count();
    }

    template<class T1, class T2, class T3>
    QList<T1> ObjectTable<T1,T2,T3>::keys()
    {
        return objectTable.keys();
    }

    template<class T1, class T2, class T3>
    bool ObjectTable<T1,T2,T3>::containsRow(T1 row)
    {
        return objectTable.contains(row);
    }
}
#endif // OBJECTTABLE_H
