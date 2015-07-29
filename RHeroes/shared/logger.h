#ifndef LOGGER_H
#define LOGGER_H

#include <QHash>
#include <QMutex>
#include <QMap>
#include <QRegExp>
#include <iostream>

class LogIndex;
class LoggerStream;

class Logger
{
public:
    enum OutputFormat {
        PrependAll,
        PrependTimeLevelFile,
        PrependTimeFileLine,
        PrependLevelFileLine,
        PrependTimeFile,
        PrependTimeLevel,
        PrependFileLine,
        PrependLevelFile,
        PrependTime,
        PrependLevel,
        PrependFile,
        PrependCustom,
        PrependNothing
    };

    enum LogLevel {
        Debug = 0,
        Print = 1,
        Warning = 2,
        Error = 3,
        Critical = 4
    };

    enum {
        OutputCodeAll = 0,
        OutputCodeSTDOUT = 1,
        OutputCodeSTDERR = 2
    };

    Logger();
    virtual ~Logger();

    int addSTDOUTOutput();
    int addSTDERROutput();
    int addFileOutput(const QString &filename, bool append = true);
    void removeOutput(int code = OutputCodeAll);
    void setFilenameFilter(const QString &filename, int code = OutputCodeAll);
    void setFilenameFilter(const QRegExp &regexp, int code = OutputCodeAll);
    void unsetFilenameFilter(int code = OutputCodeAll);
    void setLogLevelFilter(LogLevel minlevel, int code = OutputCodeAll);

    void setOutputFormat(OutputFormat format);
    void setCustomOutputFormat(const QString &format);
    void setQTStackTrace(bool print = true);

    LoggerStream &getStream(const char *srcfile, int line, LogLevel loglevel);

private:
    void closeFiles();
    void write(const char *str, const char *srcfile, LogLevel loglevel);
    void flush(const char *srcfile, LogLevel loglevel);
    int genNewCode();

    QHash<int, QRegExp> matchers;
    QHash<int, int> minloglevels;
    QHash<int, std::ostream *> trueStreams;

    OutputFormat format;
    QHash<LogIndex, LoggerStream> streams;
    QHash<OutputFormat, QString> prependFormats;

    bool printstack;
    QMutex mutex;

    friend class LoggerStream;
    friend void qtMessageHandler(QtMsgType msg, const char *text);
};

class LogIndex {
public:
    inline LogIndex(const QString &fname, Logger::LogLevel level) :
        fname(fname), level(level) {}

    inline bool operator==(const LogIndex &l) const {
        return fname == l.fname && level == l.level;
    }

    inline const QString &fileName() const { return fname; }
    inline Logger::LogLevel logLevel() const { return level; }

private:
    QString fname; Logger::LogLevel level;

    friend uint qHash(const LogIndex &l);
};

inline uint qHash(const LogIndex &l) {
    return qHash(l.fname) * (int) l.level;
}

#ifdef __DEBUG__
#   define ldbg (logger.getStream(__FILE__, __LINE__, Logger::Debug))
#else
#   define ldbg (__fake_logger_stream)
#endif
#define lprint  (logger.getStream(__FILE__, __LINE__, Logger::Print))
#define lwarn   (logger.getStream(__FILE__, __LINE__, Logger::Warning))
#define lerr    (logger.getStream(__FILE__, __LINE__, Logger::Error))
#define lcrit   (logger.getStream(__FILE__, __LINE__, Logger::Critical))

extern Logger logger;

void printStackTrace(LoggerStream &stream);

#ifndef __DEBUG__
class FakeLoggerStream
{
public:
    inline void setLine(int line) { Q_UNUSED(line) }
    inline void setLogLevel(int level) { Q_UNUSED(level) }
    inline void writeAtomic(const char *text) { Q_UNUSED(text) }
    inline void write(const char *text) { Q_UNUSED(text) }
    inline FakeLoggerStream &flush() { return *this; }
    inline FakeLoggerStream &endl() { return *this; }
    inline FakeLoggerStream &clear() { return *this; }

    template <typename T>
    inline FakeLoggerStream &operator<<(T x) { Q_UNUSED(x) return *this;}

    typedef FakeLoggerStream &(*FakeLoggerStreamManipulator)(FakeLoggerStream &);
    inline FakeLoggerStream &operator<<(FakeLoggerStreamManipulator manip) {
        Q_UNUSED(manip) return *this; }
};

inline FakeLoggerStream &flush(FakeLoggerStream &stream) { return stream; }
inline FakeLoggerStream &endl(FakeLoggerStream &stream) { return stream; }
inline FakeLoggerStream &clear(FakeLoggerStream &stream) { return stream; }
inline void printStackTrace(FakeLoggerStream &stream) { Q_UNUSED(stream) }

extern FakeLoggerStream __fake_logger_stream;
#endif

#include "loggerstream.h"

#endif // LOGGER_H
