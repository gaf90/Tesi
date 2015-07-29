#include "loggerstream.h"
#include <QDateTime>
#include <QStringList>
#include <QTextStream>
#include <QThread>

const QString
    LoggerStream::logLevelDebug = "Debug",
    LoggerStream::logLevelPrint = "Print",
    LoggerStream::logLevelWarning = "Warning",
    LoggerStream::logLevelError = "Error",
    LoggerStream::logLevelCritical = "Critical";

static const char *fmt_regexp =
        "^(.*)([%]*)(%)([-+#0 'I]?)([0-9]*|\\*?)((?:\\.[0-9]+|\\.)?)"
        "(?:hh|h|l|ll|j|z|t|L)?([cdiouxXeEfFaAgGsp])(.*)$";

static const char *str_newline = "\n";

static int lastWriteLevel = -1;
static bool newline = true;

LoggerStream::LoggerStream() :
    srcfile(""), regexp(fmt_regexp), level(Logger::Debug)
{
    init();
}

LoggerStream::LoggerStream(const char *srcfile, Logger::LogLevel level) :
    srcfile(srcfile), regexp(fmt_regexp), level(level)
{
    init();
}

LoggerStream::LoggerStream(const LoggerStream &lstream)
{
    *this = lstream;
}

LoggerStream &LoggerStream::operator=(const LoggerStream &lstream)
{
    mutex.lock();
    srcfile = lstream.srcfile;
    srcfilename = lstream.srcfilename;
    regexp = QRegExp(fmt_regexp);
    level = lstream.level;
    regexp.setMinimal(true);
    for(int i = 0; i < LOGGER_STREAM_NFORMATS; i++) {
        fmts[i] = strdup(lstream.fmts[i]);
        specifiers[i] = lstream.specifiers[i];
    }
    mutex.unlock();
    return *this;
}

LoggerStream::~LoggerStream()
{
    for(int i = 0; i < LOGGER_STREAM_NFORMATS; i++) {
        free(fmts[i]);
    }
}

void LoggerStream::init()
{
	const char *str1 = strrchr(srcfile, '/'), *str2 = strrchr(srcfile, '\\');
	if(str1 == NULL && str2 == NULL) {
		srcfilename = srcfile;
	} else if(str1 > str2) {
		srcfilename = str1 + 1;
	} else {
		srcfilename = str2 + 1;
	}

    regexp.setMinimal(true);
    fmts[0] = strdup("%f");     specifiers[0] = 'f';    // float, double
    fmts[1] = strdup("%Lf");    specifiers[1] = 'f';    // long double

    fmts[2] = strdup("%hd");    specifiers[2] = 'd';    // signed short
    fmts[3] = strdup("%d");     specifiers[3] = 'd';    // signed int
    fmts[4] = strdup("%ld");    specifiers[4] = 'd';    // signed long
    fmts[5] = strdup("%lld");   specifiers[5] = 'd';    // signed long long

    fmts[6] = strdup("%hu");    specifiers[6] = 'u';    // unsigned short
    fmts[7] = strdup("%u");     specifiers[7] = 'u';    // unsigned int
    fmts[8] = strdup("%lu");    specifiers[8] = 'u';    // unsigned long
    fmts[9] = strdup("%llu");   specifiers[9] = 'u';    // unsigned long long

    fmts[10] = strdup("%c");
}

void LoggerStream::setLine(int line)
{
    mutex.lock();
    currentLine = line;
    mutex.unlock();
}

void LoggerStream::writeForwarder(const char *text, bool doflush)
{
    if(text[0] != '\0' || (doflush && buffer.size() > 0)) {
        if(!doflush && buffer.size() < 64 * 1024 && text[0] != '\n') {
            buffer.append(text);
        } else {
            if(newline) {
                formatPrepend();
                newline = false;
            } else if(lastWriteLevel != level) {
                formatPrepend();
                buffer.prepend(str_newline);
            }

            if(text[0] == '\n') {
                newline = true;
                buffer.append(str_newline);
            } else {
                buffer.append(text);
            }

            lastWriteLevel = level;
            logger.write(buffer.toLatin1().data(), srcfile, level);
            buffer.clear();
        }
    }
}

void LoggerStream::write(const char *text)
{
    logger.mutex.lock();
    QString qtext = QString(text).remove('\r');
    QStringList l = qtext.split('\n');
    for(int i = 0; i < l.size() - 1; i++) {
        QString &elem = l[i];
        if(!l.isEmpty()) writeForwarder(elem.toLatin1().data());
        writeForwarder("\n");
    }
    if(!l.last().isEmpty())
        writeForwarder(l.last().toLatin1().data());
    logger.mutex.unlock();
}

LoggerStream &LoggerStream::flush() {
    logger.mutex.lock();
    writeForwarder("", true);
    logger.flush(srcfile, level);
    logger.mutex.unlock();
    return *this;
}

LoggerStream &LoggerStream::endl() {
    logger.mutex.lock();
    writeForwarder("\n");
    logger.flush(srcfile, level);
    logger.mutex.unlock();
    return *this;
}

LoggerStream &LoggerStream::clear() {
    buffer.clear();
    return *this;
}

LoggerStream &LoggerStream::lastEndl() {
    if(buffer.size() > 0) {
        endl();
    }
    return *this;
}

const QString &LoggerStream::logLevelText(Logger::LogLevel l) {
    switch(l) {
    case Logger::Debug:     return LoggerStream::logLevelDebug;
    case Logger::Print:     return LoggerStream::logLevelPrint;
    case Logger::Warning:   return LoggerStream::logLevelWarning;
    case Logger::Error:     return LoggerStream::logLevelError;
    default:                return LoggerStream::logLevelCritical;
    }
}

void LoggerStream::formatPrepend()
{
    QString base = logger.prependFormats[logger.format];
    char *time;
    asprintf(&time, "%.3f", QDateTime::currentMSecsSinceEpoch() * .001);
    base.replace("%F", srcfilename);
    base.replace("%N", QString::number(currentLine));
    base.replace("%L", logLevelText(level));
    base.replace("%T", time);
    if(base.contains("%Q")) {
        Qt::HANDLE threadId = QThread::currentThreadId();
        QString formatted("");
#if defined(Q_OS_MAC) || defined(Q_OS_WIN32) || defined(Q_OS_UNIX)
        char *ptr = formatPointer(threadId);
        formatted = ptr;
        free(ptr);
#endif
        base.replace("%Q", formatted);
    }
    buffer.prepend(base);
    free(time);
}

static void adjustfmtSingle(
        const QString &before,
        const QString &after,
        const QString &flags,
        const QString &minwidth,
        const QString &precision,
        const QString &specifier,
        const QString &length,
        bool isreal, bool issigned,
        char *&newfmt, char &newspec)
{
    static const QString realSpecifiers     = "eEfFaAgGs";
    static const QString signedSpecifiers   = "di";
    static const QString unsignedSpecifiers = "ouxX";
    QString middle = "%" + flags + minwidth + precision + length;

    if(isreal) {
        if(realSpecifiers.indexOf(specifier) != -1) {
            middle += specifier; newspec = specifier.at(0).toLatin1();
        } else {
            middle += "f"; newspec = 'f';
        }
    } else if(issigned) {
        if(signedSpecifiers.indexOf(specifier) != -1 ||
                unsignedSpecifiers.indexOf(specifier) != -1) {
            middle += specifier; newspec = specifier.at(0).toLatin1();
        } else {
            middle += "d"; newspec = 'd';
        }
    } else {
        if(unsignedSpecifiers.indexOf(specifier) != -1) {
            middle += specifier; newspec = specifier.at(0).toLatin1();
        } else {
            middle += "u"; newspec = 'u';
        }
    }

    QString total = before + middle + after;

    free(newfmt);
    newfmt = strdup(total.toLatin1().data());
}

#define quickAdjust(a, b, c, d) \
    adjustfmtSingle( \
        before, after, flags, minwidth, precision, \
        specifier, a, b, c, fmts[d], specifiers[d])

void LoggerStream::adjustfmt(const char *basefmt, bool real)
{
    mutex.lock();
    QString base(basefmt), partial(base);
    int start = 0, len;
    bool matched = regexp.exactMatch(base), found = false;
    while((len = regexp.matchedLength()) > 0) {
        if(matched && regexp.cap(1).length() % 2 == 0) {
            found = true;
            break;
        } else {
            start += len;
            partial = base.mid(len);
            matched = regexp.exactMatch(partial);
        }
    }
    if(found) {
        QString before = cleanPercent(base.left(start + regexp.pos(3))),
                after  = cleanPercent(regexp.cap(8));
        QString flags = regexp.cap(4),
                minwidth = regexp.cap(5),
                precision = regexp.cap(6),
                specifier = regexp.cap(7);

        if(real) {
            quickAdjust("", true, false, 0);       // float, double
            quickAdjust("L", true, false, 1);      // long double
        } else {
            quickAdjust("h", false, true, 2);      // signed short
            quickAdjust("", false, true, 3);       // signed int
            quickAdjust("l", false, true, 4);      // signed long
            quickAdjust("ll", false, true, 5);     // signed long long

            quickAdjust("h", false, false, 6);     // unsigned short
            quickAdjust("", false, false, 7);      // unsigned int
            quickAdjust("l", false, false, 8);     // unsigned long
            quickAdjust("ll", false, false, 9);    // unsigned long long
        }
    } else {
        QString fmtConst = cleanPercent(base);
        for(int i = 0; i < LOGGER_STREAM_NFORMATS; i++) {
            free(fmts[i]);
            fmts[i] = strdup(fmtConst.toLatin1().data());
        }
    }
    mutex.unlock();
}

#undef quickAdjust

QString LoggerStream::cleanPercent(const QString &str)
{
    QString ret(str);
    return ret.replace("%%", "%").replace("%", "%%");
}

template <typename T, int I, bool C>
LoggerStream &LoggerStream::formatWrite(T value)
{
    logger.mutex.lock();
    char *data, *comma;
    asprintf(&data, fmts[I], value);
    if(C && (comma = strchr(data, ',')) != NULL) {
        /* I'd love if OSX would stop ignoring my setlocale(LC_NUMERIC, "en") instead
           of having to write this ugly hack */
        comma[0] = '.';
    }
    writeForwarder(data);
    free(data);
    logger.mutex.unlock();
    return *this;
}


void LoggerStream::writePrintableString(const QByteArray &data)
{
    const int bufsize = 4096;
    const char hex[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
    };
    char buf[bufsize];
    int j = 0;
    for(int i = 0; i < data.size(); i++) {
        unsigned char c = data[i];
        if(j > bufsize - 5) {
            buf[j] = '\0';
            write(buf);
            j = 0;
        }
        if(c >= 0x20 && c <= 0x7E) {
            buf[j++] = c;
        } else {
            buf[j++] = '\\';
            buf[j++] = 'x';
            buf[j++] = hex[(c >> 4) & 0x0F];
            buf[j++] = hex[c & 0x0F];
        }
    }
    if(j > 0) {
        buf[j] = '\0';
        write(buf);
    }
}

LoggerStream &LoggerStream::operator<<(const char *arg) {
    write(arg); return *this; }
LoggerStream &LoggerStream::operator<<(const QString &arg) {
    write(arg.toLatin1().data()); return *this; }
LoggerStream &LoggerStream::operator<<(const QByteArray &arg) {
    writePrintableString(arg); return (*this); }
LoggerStream &LoggerStream::operator<<(const std::string &arg) {
    write(arg.c_str()); return *this; }

LoggerStream &LoggerStream::operator<<(bool arg) {
    write(arg ? "true" : "false"); return *this; }
LoggerStream &LoggerStream::operator<<(char arg) {
    return formatWrite<char, 10, false>(arg); }
LoggerStream &LoggerStream::operator<<(QChar arg) {
    return (*this) << arg.toLatin1(); }

LoggerStream &LoggerStream::operator<<(float arg) {
    return formatWrite<float, 0, true>(arg); }
LoggerStream &LoggerStream::operator<<(double arg) {
    return formatWrite<double, 0, true>(arg); }
LoggerStream &LoggerStream::operator<<(long double arg) {
    return formatWrite<long double, 1, true>(arg); }

LoggerStream &LoggerStream::operator<<(signed short arg) {
    return formatWrite<signed short int, 2, false>(arg); }
LoggerStream &LoggerStream::operator<<(signed int arg) {
    return formatWrite<signed int, 3, false>(arg); }
LoggerStream &LoggerStream::operator<<(signed long arg) {
    return formatWrite<signed long, 4, false>(arg); }
LoggerStream &LoggerStream::operator<<(signed long long arg) {
    return formatWrite<signed long long, 5, false>(arg); }

LoggerStream &LoggerStream::operator<<(unsigned short arg) {
    return formatWrite<unsigned short int, 6, false>(arg); }
LoggerStream &LoggerStream::operator<<(unsigned int arg) {
    return formatWrite<unsigned int, 7, false>(arg); }
LoggerStream &LoggerStream::operator<<(unsigned long arg) {
    return formatWrite<unsigned long, 8, false>(arg); }
LoggerStream &LoggerStream::operator<<(unsigned long long arg) {
    return formatWrite<unsigned long long, 9, false>(arg); }

LoggerStream &LoggerStream::operator<<(realfmt fmt) {
    adjustfmt(fmt.fmt, true); return *this; }
LoggerStream &LoggerStream::operator<<(intfmt fmt) {
    adjustfmt(fmt.fmt, false); return *this; }
LoggerStream &LoggerStream::operator<<(LoggerStreamManipulator manip) {
    return (*manip)(*this); }
