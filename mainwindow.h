#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <QImage>
#include <QThread>
#include <QDir>

#include "common.h"
#include "imageview.h"
#include "landmark_detector.h"
#include "gaze_tracker.h"
#include "hand_detector.hpp"
#include "kalman_tracker.h"
#include "ambp_engine.h"
#include "framediff_engine.h"

class ProcessThread : public QObject
{
    Q_OBJECT

public:
    ProcessThread(QObject* parent = 0);
    ~ProcessThread();

public Q_SLOTS:
        void start();

Q_SIGNALS:
        void pathFound(const QString& path);

        void finished();

        void requestFinish();

		void requestSurveilanceCnt(int value);

		void requestUpCnt(int value);

		void requestDownCnt(int value);

private:
        void isFinished(){ emit requestFinish(); }

		void setSurveilanceCnt(int value){ emit requestSurveilanceCnt(value); }

		void setUpCnt(int value){ emit requestUpCnt(value); }

		void setDownCnt(int value){ emit requestDownCnt(value); }

        // background modeling
        void createBgEngine();

        void deleteBgEngine();

        void backgroundModeling(Mat procImg);

        void removeNoise();

        // object detecting
        void detectObjects();

        void releaseObjectBuffer(FRAME_OBJECTS *buffer);

        /* kalman tracking */
        void kalman_tracking();

		/* meanshift tracking */
		void meanshift_tracking();

		void getMeanshiftObjectID();

		void checkState();

public:
        QLabel *m_pVideoView;

        VideoCapture *m_capture;

        QString m_strVideoName;

        int m_nInputType, m_nProcessMode, m_nMinSize, m_nMaxSize, m_nSegmentThreshold;

private:
        Mat m_currentFrame, m_currentFg, m_currentBg;

		AMBPHistogram m_histogram;

        FrameDiff_Engine *m_pFrameDiffEngine;

        AMBP_engine *m_pAMBPEngine;

        Mat ambp_fg, framediff_fg;

		int m_nFrameNum;

        // object detecting
        vector< vector<Point> > m_contours;

        vector<Vec4i> m_hierarchy;

		/* kalman tracking */
		KalmanTracker *kalman_tracker;

		vector<Point2d> m_kalmanObjectCenters;

		vector<Rect> m_kalmanObjectRects;

		/* object tracking */
		FRAME_OBJECTS m_currentObjects, m_beforeObjects, m_lastObjects, m_thirdObjects;

		int m_nObjectMaxID, m_nUpCnt, m_nDownCnt, m_nDoorPosition;

		Rect m_nSurveilanceArea;

		vector<int> m_vecProcessedIDs;

		vector<OBJECT> m_vecFirstDatas;
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_webcam_radio_clicked(bool checked);

    void on_video_radio_clicked(bool checked);

    void on_video_open_btn_clicked();

    void on_min_size_slider_valueChanged(int value);

    void on_max_size_slider_valueChanged(int value);

    void on_segment_threshold_slider_valueChanged(int value);

    void on_process_btn_clicked();

    void on_stop_btn_clicked();

    void on_process_method_combo_activated(int index);

    void on_thread_finished();

	void on_surveilace_request(int value);

	void on_up_request(int value);

	void on_down_request(int value);

private:
    void InitUI();

private:
    Ui::MainWindow *ui;

    QString m_strVideoFileName;

    ProcessThread *m_pProcessThread;

    QThread *m_pThread;

    int m_nInputType, m_nProcessMode, m_nMinSize, m_nMaxSize, m_nSegmentThreshold;
};

#endif // MAINWINDOW_H
