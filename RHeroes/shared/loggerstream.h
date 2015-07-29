#ifndef LOGGERSTREAM_H
#define LOGGERSTREAM_H

#include <QString>
#include <QList>
#include <QVector>
#include <QHash>
#include <QRegExp>
#include <QMutex>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <list>
#include <deque>
#include <complex>
#include <stdio.h>
#include <Eigen/Core>
#include "logger.h"

/* Magnificent, Windows has no asprintf function. */
#ifdef Q_OS_WIN32
extern "C" {
    int asprintf(char **buffer, const char *fmt, ...);
}
#endif

#define LOGGER_STREAM_NFORMATS 11

class realfmt;
class intfmt;
template <typename T> class arraytraits;

class LoggerStream {
public:
    LoggerStream();
    LoggerStream(const char *srcfile, Logger::LogLevel level);
    LoggerStream(const LoggerStream &lstream);
    virtual ~LoggerStream();

    void setLine(int line);
    void setSourceFilename(const char *filename);
    void write(const char *text);
    LoggerStream &flush();
    LoggerStream &endl();
    LoggerStream &clear();
    LoggerStream &lastEndl();

    LoggerStream &operator=(const LoggerStream &lstream);

    LoggerStream &operator<<(const char *arg);
    LoggerStream &operator<<(const QString &arg);
    LoggerStream &operator<<(const QByteArray &arg);
    LoggerStream &operator<<(const std::string &arg);

    LoggerStream &operator<<(bool arg);
    LoggerStream &operator<<(char arg);
    LoggerStream &operator<<(QChar arg);

    LoggerStream &operator<<(float arg);
    LoggerStream &operator<<(double arg);
    LoggerStream &operator<<(long double arg);

    LoggerStream &operator<<(signed short arg);
    LoggerStream &operator<<(signed int arg);
    LoggerStream &operator<<(signed long arg);
    LoggerStream &operator<<(signed long long arg);

    LoggerStream &operator<<(unsigned short arg);
    LoggerStream &operator<<(unsigned int arg);
    LoggerStream &operator<<(unsigned long arg);
    LoggerStream &operator<<(unsigned long long arg);

    LoggerStream &operator<<(realfmt fmt);
    LoggerStream &operator<<(intfmt fmt);

    template <typename T> LoggerStream &operator<<(const std::complex<T> &arg);

    template <typename T> LoggerStream &operator<<(const T *arg);

    template <typename T, int N> LoggerStream &operator<<(const T (&arg)[N]);
    template <typename T> LoggerStream &operator<<(arraytraits<T> arg);

    template <typename T> LoggerStream &operator<<(const QList<T> &arg);
    template <typename T> LoggerStream &operator<<(const QLinkedList<T> &arg);
    template <typename T> LoggerStream &operator<<(const QVector<T> &arg);

    template <typename T> LoggerStream &operator<<(const std::vector<T> &arg);
    template <typename T> LoggerStream &operator<<(const std::list<T> &arg);
    template <typename T> LoggerStream &operator<<(const std::deque<T> &arg);

    template <typename K, typename T> LoggerStream &operator<<(const QHash<K, T> &arg);
    template <typename K, typename T> LoggerStream &operator<<(const std::pair<K, T> &arg);

    template <typename Derived>
    LoggerStream &operator<<(const Eigen::DenseBase<Derived> &mat);

    typedef LoggerStream &(*LoggerStreamManipulator)(LoggerStream &);
    LoggerStream &operator<<(LoggerStreamManipulator manip);

    template <typename Iterator> LoggerStream &sharedListOperator(Iterator first, Iterator last);

private:
    void init();
    const QString &logLevelText(Logger::LogLevel l);
    void writeForwarder(const char *text, bool doflush = false);
    void formatPrepend();
    void adjustfmt(const char *basefmt, bool real);
    QString cleanPercent(const QString &str);
    void writePrintableString(const QByteArray &data);

    template <typename T> char *formatPointer(T arg);
    template <typename T, int I, bool C> LoggerStream &formatWrite(T value);

    const char *srcfile, *srcfilename;
    char *fmts[LOGGER_STREAM_NFORMATS], specifiers[LOGGER_STREAM_NFORMATS];
    QRegExp regexp;
    QMutex mutex;
    int currentLine;
    Logger::LogLevel level;
    QString buffer;

    /* Static constants/values */
    static const QString
        logLevelDebug,
        logLevelPrint,
        logLevelWarning,
        logLevelError,
        logLevelCritical;

};

class realfmt {
public:
    inline realfmt(const char *fmt) : fmt(fmt) {}
private:
    const char *fmt;
    friend class LoggerStream;
};

class intfmt {
public:
    inline intfmt(const char *fmt) : fmt(fmt) {}
private:
    const char *fmt;
    friend class LoggerStream;
};

template <typename T>
class arraytraits {
public:
	inline arraytraits(const T *array, int size) : array(array), size(size) {}
private:
	const T *array;
	int size;
	friend class LoggerStream;
};

inline LoggerStream &flush(LoggerStream &stream) { return stream.flush(); }
inline LoggerStream &endl(LoggerStream &stream) { return stream.endl(); }
inline LoggerStream &clear(LoggerStream &stream) { return stream.clear(); }

template <typename T>
inline arraytraits<T> arrayprint(const T *array, int size) {
	return arraytraits<T>(array, size);
}

template <typename T>
char *LoggerStream::formatPointer(T arg) {
    char *ptr;
#if QT_POINTER_SIZE == 4
    asprintf(&ptr, "0x%08x", (quint32) arg);
#else
    asprintf(&ptr, "0x%016llx", (quint64) arg);
#endif
    return ptr;
}

template <typename Iterator>
LoggerStream &LoggerStream::sharedListOperator(Iterator first, Iterator last) {
    (*this) << "{";
    while(first != last) {
        (*this) << *first;
        ++first;
        if(first != last)
            (*this) << ", ";
    }
    (*this) << "}";
    return *this;
}


template <typename T>
LoggerStream &LoggerStream::operator<<(const std::complex<T> &arg) {
    return (*this) << "(" << arg.real() << "," << arg.imag() << ")";
}

template <typename T>
LoggerStream &LoggerStream::operator<<(const T *arg) {
    char *ptr = formatPointer(arg);
    (*this) << ptr;
    free(ptr);
    return (*this);
}

template <typename T, int N>
LoggerStream &LoggerStream::operator<<(const T (&arg)[N]) {
	return sharedListOperator(arg, arg + N); }
template <typename T>
LoggerStream &LoggerStream::operator<<(arraytraits<T> traits) {
	return sharedListOperator(traits.array, traits.array + traits.size); }
template <typename T>
LoggerStream &LoggerStream::operator<<(const QList<T> &arg) {
    return sharedListOperator(arg.begin(), arg.end()); }
template <typename T>
LoggerStream &LoggerStream::operator<<(const QLinkedList<T> &arg) {
    return sharedListOperator(arg.begin(), arg.end()); }
template <typename T>
LoggerStream &LoggerStream::operator<<(const QVector<T> &arg) {
    return sharedListOperator(arg.begin(), arg.end()); }
template <typename T>
LoggerStream &LoggerStream::operator<<(const std::vector<T> &arg) {
    return sharedListOperator(arg.begin(), arg.end()); }
template <typename T>
LoggerStream &LoggerStream::operator<<(const std::list<T> &arg) {
    return sharedListOperator(arg.begin(), arg.end()); }
template <typename T>
LoggerStream &LoggerStream::operator<<(const std::deque<T> &arg) {
    return sharedListOperator(arg.begin(), arg.end()); }

template <typename K, typename T>
LoggerStream &LoggerStream::operator<<(const std::pair<K, T> &arg) {
    return (*this) << "{" << arg.first << "," << arg.second << "}";
}

template <typename K, typename T>
LoggerStream &LoggerStream::operator<<(const QHash<K, T> &arg) {
    typename QHash<K, T>::const_iterator it = arg.begin();
    (*this) << "{";
    if(it != arg.end()) (*this) << it.key() << ": " << it.value();
    ++it;
    while(it != arg.end()) {
        (*this) << ", " << it.key() << ": " << it.value();
        ++it;
    }
    return (*this) << "}";
}

template <typename Derived>
LoggerStream &LoggerStream::operator<<(const Eigen::DenseBase<Derived> &mat) {
	(*this) << "{";
	for(int i = 0; i < mat.rows(); i++) {
		(*this) << "{";
		for(int j = 0; j < mat.cols(); j++) {
			(*this) << mat(i, j);
			if(j < mat.cols() - 1) (*this) << ",";
		}
		(*this) << "}";
		if(i < mat.rows() - 1) (*this) << ",";
	}
	return (*this) << "}";
}


#endif // LOGGERSTREAM_H
