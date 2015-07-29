#-------------------------------------------------
#
# Project created by Mladen destroyer of source
#
#-------------------------------------------------

SOURCES += \
    shared/loggerstream.cpp \
    shared/logger.cpp \
    shared/config.cpp \
    shared/configreader.cpp \
    shared/random.cpp
    
win32:SOURCES += \
    shared/asprintf.c \
    shared/winbacktrace.c

HEADERS += \
    shared/constants.h \
    shared/utilities.h \
    shared/mutexqueue.h \
    shared/loggerstream.h \
    shared/logger.h \
    shared/config.h \
    shared/configreader.h \
    shared/doubleqhash.h \
    shared/doubleqhash.h \
    shared/random.h \
    shared/stepwisecdf.h

win32:HEADERS += shared/winbacktrace.h
