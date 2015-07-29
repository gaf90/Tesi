#include "logger.h"
#include <fstream>
#include <stdlib.h>

#include <cxxabi.h>
#if defined(Q_OS_WIN32)
#   include <windows.h>
#   include "winbacktrace.h"
#elif defined(Q_OS_UNIX)
#   include <execinfo.h>
#   include <signal.h>
#endif

Logger logger;

#ifndef __DEBUG__
    FakeLoggerStream __fake_logger_stream;
#endif

#if defined(Q_OS_MAC)
    QString osxFormat(const QString &str, int size);
#elif defined(Q_OS_UNIX)
    QString unixFormat(const QString &str, int i, int size);
#endif

#if defined(Q_OS_WIN32)
    static LPTOP_LEVEL_EXCEPTION_FILTER prevExceptionFilter = NULL;
    LONG WINAPI exceptionFilter(LPEXCEPTION_POINTERS info);
#elif defined(Q_OS_UNIX)
    void sigsegvHandler(int code);
#endif

#define MAX_BACKTRACE 128

void qtMessageHandler(QtMsgType msg, const char *text);
QString demangle(const QString &mangled);

Logger::Logger() : format(Logger::PrependFileLine), printstack(true)
{
    prependFormats.insert(Logger::PrependAll,           "[%T %L:%F@%N]: ");
    prependFormats.insert(Logger::PrependTimeLevelFile, "[%T %L:%F]: ");
    prependFormats.insert(Logger::PrependTimeFileLine,	"[%T %F@%N]: ");
    prependFormats.insert(Logger::PrependLevelFileLine, "[%L:%F@%N]: ");
    prependFormats.insert(Logger::PrependTimeFile,      "[%T %F]: ");
    prependFormats.insert(Logger::PrependTimeLevel,     "[%T %L]: ");
    prependFormats.insert(Logger::PrependFileLine,      "[%F@%N]: ");
    prependFormats.insert(Logger::PrependLevelFile,     "[%L:%F]: ");
    prependFormats.insert(Logger::PrependTime,          "[%T]: ");
    prependFormats.insert(Logger::PrependLevel,         "[%L]: ");
    prependFormats.insert(Logger::PrependFile,          "[%F]: ");
    prependFormats.insert(Logger::PrependCustom,        "[%L:%F@%L]: ");
    prependFormats.insert(Logger::PrependNothing,       "");

    qInstallMsgHandler(qtMessageHandler);

    trueStreams[Logger::OutputCodeSTDOUT] = &std::cout;
    matchers[Logger::OutputCodeSTDOUT] = QRegExp(".*");
    minloglevels[Logger::OutputCodeSTDOUT] = 0;


#if defined(Q_OS_WIN32)
    prevExceptionFilter = SetUnhandledExceptionFilter(exceptionFilter);
#elif defined(Q_OS_UNIX)
    signal(SIGSEGV, sigsegvHandler);
#endif
}

Logger::~Logger()
{
    QHash<LogIndex, LoggerStream>::iterator it = streams.begin();
    while(it != streams.end()) {
        it.value().lastEndl();
        ++it;
    }
    closeFiles();
#if defined(Q_OS_WIN32)
    SetUnhandledExceptionFilter(prevExceptionFilter);
#endif
}

void Logger::setOutputFormat(Logger::OutputFormat format)
{
    mutex.lock();
    this->format = format;
    mutex.unlock();
}

void Logger::setCustomOutputFormat(const QString &format)
{
    mutex.lock();
    prependFormats[Logger::PrependCustom] = format;
    mutex.unlock();
}

int Logger::addSTDOUTOutput()
{
    mutex.lock();
    if(!trueStreams.contains(Logger::OutputCodeSTDOUT)) {
        trueStreams[Logger::OutputCodeSTDOUT] = &std::cout;
        matchers[Logger::OutputCodeSTDOUT] = QRegExp(".*");
        minloglevels[Logger::OutputCodeSTDOUT] = 0;
    }
    mutex.unlock();
    return Logger::OutputCodeSTDOUT;
}

int Logger::addSTDERROutput()
{
    mutex.lock();
    if(!trueStreams.contains(Logger::OutputCodeSTDERR)) {
        trueStreams[Logger::OutputCodeSTDERR] = &std::cerr;
        matchers[Logger::OutputCodeSTDERR] = QRegExp(".*");
        minloglevels[Logger::OutputCodeSTDERR] = 0;
    }
    mutex.unlock();
    return Logger::OutputCodeSTDERR;
}

int Logger::genNewCode()
{
    int ret;
    do {
        ret = rand();
    } while (ret <= 2 || trueStreams.contains(ret));
    return ret;
}

int Logger::addFileOutput(const QString &filename, bool append)
{
    mutex.lock();
    std::ios_base::openmode mode;

    if(append) {
        mode = std::ios::out | std::ios::app;
    } else {
        mode = std::ios::out;
    }

    int code = genNewCode();
    trueStreams[code] = new std::ofstream(filename.toLatin1().data(), mode);
    matchers[code] = QRegExp(".*");
    minloglevels[code] = 0;
    mutex.unlock();

    return code;
}

void Logger::removeOutput(int code)
{
    mutex.lock();
    if(trueStreams.contains(code)) {
        if(code > 2) {
            std::ofstream *file = (std::ofstream *) trueStreams[code];
            file->close();
            delete file;
        }
        trueStreams.remove(code);
        matchers.remove(code);
        minloglevels.remove(code);
    } else if(code == Logger::OutputCodeAll) {
        QHash<int, std::ostream *>::iterator it = trueStreams.begin();
        while(it != trueStreams.end()) {
            if(it.key() > 2) {
                std::ofstream *file = (std::ofstream *) *it;
                file->close();
                delete file;
            }
            trueStreams.remove(it.key());
            matchers.remove(it.key());
            minloglevels.remove(it.key());
            ++it;
        }
    }
    mutex.unlock();
}

void Logger::setLogLevelFilter(LogLevel minlevel, int code)
{
    mutex.lock();
    if(minloglevels.contains(code)) {
        minloglevels[code] = minlevel;
    } else if(code == Logger::OutputCodeAll) {
        QHash<int, int>::iterator it = minloglevels.begin();
        while(it != minloglevels.end()) {
            *it = minlevel;
            ++it;
        }
    }
    mutex.unlock();
}

void Logger::setFilenameFilter(const QString &filename, int code)
{
    mutex.lock();
    if(matchers.contains(code)) {
        matchers[code] = QRegExp("^" + QRegExp::escape(filename) + "$");
    } else if(code == Logger::OutputCodeAll) {
        QHash<int, QRegExp>::iterator it = matchers.begin();
        while(it != matchers.end()) {
            *it = QRegExp("^" + QRegExp::escape(filename) + "$");
            ++it;
        }
    }
    mutex.unlock();
}

void Logger::setFilenameFilter(const QRegExp &filenameRegexp, int code)
{
    mutex.lock();
    if(matchers.contains(code)) {
        matchers[code] = filenameRegexp;
    } else if(code == Logger::OutputCodeAll) {
        QHash<int, QRegExp>::iterator it = matchers.begin();
        while(it != matchers.end()) {
            *it = filenameRegexp;
            ++it;
        }
    }
    mutex.unlock();
}

void Logger::unsetFilenameFilter(int code)
{
    mutex.lock();
    if(matchers.contains(code)) {
        matchers[code] = QRegExp(".*");
    } else if(code == Logger::OutputCodeAll) {
        QHash<int, QRegExp>::iterator it = matchers.begin();
        while(it != matchers.end()) {
            *it = QRegExp(".*");
            ++it;
        }
    }
    mutex.unlock();
}

void Logger::setQTStackTrace(bool print) {
    mutex.lock();
    printstack = print;
    mutex.unlock();
}

void Logger::closeFiles()
{
    QHash<int, std::ostream *>::iterator it = trueStreams.begin();
    while(it != trueStreams.end()) {
        if(it.key() > 2) {
            std::ofstream *file = (std::ofstream *) *it;
            file->close();
            delete file;
        }
        ++it;
    }
}

LoggerStream &Logger::getStream(const char *srcfile, int line, Logger::LogLevel loglevel)
{
    mutex.lock();
    LogIndex key(srcfile, loglevel);
    if(streams.contains(key)) {
        LoggerStream &lstream = streams[key];
        lstream.setLine(line);
        mutex.unlock();
        return lstream;
    } else {
        streams.insert(key, LoggerStream(srcfile, loglevel));
        LoggerStream &lstream = streams[key];
        lstream.setLine(line);
        mutex.unlock();
        return lstream;
    }
}

void Logger::write(const char *str, const char *srcfile, LogLevel loglevel)
{
    // No mutex, managed at LoggerStream level
    QHash<int, QRegExp>::iterator it = matchers.begin();
    while(it != matchers.end()) {
        if(it.value().exactMatch(srcfile) && loglevel >= minloglevels[it.key()]) {
            (*trueStreams[it.key()]) << str;
        }
        ++it;
    }
}

void Logger::flush(const char *srcfile, LogLevel loglevel)
{
    // No mutex, managed at LoggerStream level
    QHash<int, QRegExp>::iterator it = matchers.begin();
    while(it != matchers.end()) {
        if(it.value().exactMatch(srcfile) && loglevel >= minloglevels[it.key()]) {
            trueStreams[it.key()]->flush();
        }
        ++it;
    }
}

void qtMessageHandler(QtMsgType msg, const char *text)
{
    static const char *names[] = {
        "QtDebug", "QtWarning", "QtCritical", "QtFatal"
    };

    Logger::LogLevel level;
    int idx;

    switch(msg) {
    case QtDebugMsg:
        level = Logger::Debug;    idx = 0; break;
    case QtWarningMsg:
        level = Logger::Warning;  idx = 1; break;
    case QtCriticalMsg:
        level = Logger::Error;    idx = 2; break;
    default: /* QtFatalMsg */
        level = Logger::Critical; idx = 3; break;
    }

    LoggerStream &ls = logger.getStream(names[idx], 0, level) << text << endl;

    if(logger.printstack && msg != QtDebugMsg) {
        printStackTrace(ls);
    }
}

#ifdef Q_OS_WIN32
void printStackTraceWin32(LoggerStream &stream, LPCONTEXT context = NULL)
{
    backtrace_result_t result;
    if(context) {
        backtrace_with_context(&result, MAX_BACKTRACE, context);
    } else {
        backtrace(&result, MAX_BACKTRACE);
    }

    for(int i = 0; i < result.count; i++) {
        QString func;
        if(result.entries[i].func_name) {
            func = demangle(result.entries[i].func_name);
            if(func == result.entries[i].func_name) {
                QString prepended(QString("_") + result.entries[i].func_name);
                func = demangle(prepended);
                if(func == prepended) {
                    func = result.entries[i].func_name;
                }
            }
        } else {
            func = "[unknown function]";
        }
        stream << QString("(%1) ").arg(i, result.count > 10 ? 2 : 1) <<
                  result.entries[i].address << ": " << func + " + " <<
                  result.entries[i].line << endl;
    }
    backtrace_free_result(&result);
}
#endif

void printStackTrace(LoggerStream &stream)
{
#ifdef Q_OS_WIN32
    printStackTraceWin32(stream);
#else
    void *stack[MAX_BACKTRACE];
    size_t size, i;
    char **strings;

    size = backtrace(stack, MAX_BACKTRACE);
    strings = backtrace_symbols(stack, size);

    for (i = 0; i < size; i++) {
#   ifdef Q_OS_MAC
        stream << osxFormat(strings[i], size) << endl;
#   else
        stream << unixFormat(strings[i], i, size) << endl;
#   endif
    }

    free(strings);
#endif
}

QString demangle(const QString &mangled) {
    QString demangled, pmangled = mangled;
    int status;
    char* demangledName;

    if ((demangledName = abi::__cxa_demangle(pmangled.toLatin1().data(),
                                             NULL, NULL, &status)) && status == 0) {
        demangled = demangledName;
    } else {
        demangled = mangled;
    }

    free(demangledName);
    return demangled;
}

#if defined(Q_OS_MAC)
QString osxFormat(const QString &str, int size) {
    QRegExp rx("([0-9]+)\\s+([^\\s]+)\\s+([^\\s]+)\\s+([^"
               "\\[\\s+]+|[^\\]]+\\])\\s*\\+\\s*([^\\s]+)");
    if(rx.exactMatch(str)) {
        return QString("(%1) %2: %3 + %4").arg(rx.cap(1), size > 10 ? 2 : 1).
                arg(rx.cap(3), demangle(rx.cap(4)), rx.cap(5));
    } else {
        return str;
    }
}
#elif defined(Q_OS_UNIX)
QString unixFormat(const QString &str, int i, int size) {
    QRegExp rx("[^\\(]+\\(([^\\)]*)\\) \\[0x([^\\]]+)\\]");
    if(rx.exactMatch(str)) {
        QRegExp rx2("([^+]+)\\+(.*)"), rx3("+(.*)");
        QString brackets = rx.cap(1), function("[unknown function]"), offset("?");
        if(rx2.exactMatch(brackets)) {
            function = rx2.cap(1);
            offset = rx2.cap(2);
        } else if(rx3.exactMatch(brackets)) {
            offset = rx3.cap(1);
        }

        return QString("(%1) 0x%2: %3 + %4").arg(QString::number(i), size > 10 ? 2 : 1).
                arg(rx.cap(2), QT_POINTER_SIZE * 2, '0').arg(demangle(function), offset);
    } else {
        return str;
    }
}
#endif

#if defined(Q_OS_WIN32)
LONG WINAPI exceptionFilter(LPEXCEPTION_POINTERS info) {
    printStackTraceWin32(lcrit, info->ContextRecord);
    exit(1);
    return 0;
}
#elif defined(Q_OS_UNIX)
void sigsegvHandler(int code) {
    Q_UNUSED(code)
    printStackTrace(lcrit);
    exit(1);
}
#endif
