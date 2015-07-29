#ifndef ABSTRACTSOCKETCONTROLLER_H
#define ABSTRACTSOCKETCONTROLLER_H

#include <QString>
#include <QTcpSocket>
#include <QHostAddress>
namespace Connection {

/**
 * @brief Shared functionalities for socket based controllers
 *
 * AbstractSocketController is a class that holds the methods and
 * functionalities shared by controllers based on socket connections
 *
 * @see USARController
 * @see UPISController
 * @see WSSController
 */
class AbstractSocketController : public QObject
{
    Q_OBJECT

protected:
    /**
     * Constructs a new AbstractSocketController
     *
     * @param parent Optional QObject parent
     */
    explicit AbstractSocketController(QObject *parent = 0);

public:
    /**
     * Destroys the AbstractSocketController
     */
    virtual ~AbstractSocketController();

signals:
    /**
     * This signal is emitted once the connection has been successfully
     * established
     */
    void sigConnected();

    /**
     * This signal is emitted in case an error happens
     *
     * @param msg The formatted error message
     */
    void sigError(const QString &msg);

    /**
     * This signal is emitted once the connection has been successfully
     * terminated
     */
    void sigDisconnected();

public slots:
    void reconnectToHost();
    /**
     * Establishes a connection to the specified host/port.
     * This method is non-blocking, hence, before any communication is
     * attempted the application should wait for the signalConnected()
     * signal
     *
     * @param address IP address or hostname of the target host
     * @param port TCP port to connect to
     */
    virtual void connectToHost(
            const QString &address,
            quint16 port);

    /**
     * Terminates the connection to the host.
     * The actual disconnection notification is emitted through the
     * sigDisconnected() signal
     */
    virtual void disconnectFromHost();

protected slots:
    /**
     * This slot should be implemented by subclasses. The slot is called every
     * time new data is available in the input buffer
     */
    virtual void invokeReadData() = 0;

private slots:
    void catchConnected();
    void catchError(QAbstractSocket::SocketError error);
    void catchDisconnected();

protected:
    /**
     * The actual socket used for connections, left protected for subclass
     * convenience
     */
    QTcpSocket *socket;
private:
    QHostAddress addr;
    quint16 port;
};

}

#endif // ABSTRACTSOCKETCONTROLLER_H
