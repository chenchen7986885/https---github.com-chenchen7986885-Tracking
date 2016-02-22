#-------------------------------------------------
#
# Project created by QtCreator 2014-11-16T21:02:30
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = Tracking_Analysis
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    gaze_tracker.cpp \
    hand_detector.cpp \
    imageview.cpp \
    kalman_tracker.cpp \
    landmark_detector.cpp \
    ambp_engine.cpp \
    ambp_objectextraction.cpp \
    framediff_engine.cpp \
    isodata_threshold.cpp

HEADERS  += mainwindow.h \
    common.h \
    gaze_tracker.h \
    hand_detector.hpp \
    imageview.h \
    kalman_tracker.h \
    landmark_detector.h \
    ambp_engine.h \
    ambp_objectextraction.h \
    framediff_engine.h \
    isodata_threshold.h

FORMS    += mainwindow.ui

INCLUDEPATH += $$PWD/library/include
DEPENDPATH += $$PWD/library/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_core2410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_core2410d
else:unix: LIBS += -L$$PWD/library/lib/ -lopencv_core2410

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_highgui2410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_highgui2410d
else:unix: LIBS += -L$$PWD/library/lib/ -lopencv_highgui2410

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_imgproc2410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_imgproc2410d
else:unix: LIBS += -L$$PWD/library/lib/ -lopencv_imgproc2410

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_video2410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_video2410d
else:unix: LIBS += -L$$PWD/library/lib/ -lopencv_video2410

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_objdetect2410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_objdetect2410d
else:unix: LIBS += -L$$PWD/library/lib/ -lopencv_objdetect2410

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_features2d2410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_features2d2410d
else:unix: LIBS += -L$$PWD/library/lib/ -lopencv_features2d2410

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_legacy2410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/library/lib/ -lopencv_legacy2410d

OTHER_FILES += \
    flandmark_model.dat \
    face.xml
