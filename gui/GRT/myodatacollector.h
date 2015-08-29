#ifndef MYODATACOLLECTOR_H
#define MYODATACOLLECTOR_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <QObject>
#include <QTimer>
#include "myo/myo.hpp"

class MyoDataCollector : public QObject, public myo::DeviceListener
{
    Q_OBJECT

    const int interval = 1000/20;

public:        
    static myo::Hub& getHub() {
        static myo::Hub hub("com.example.hello-myo");
        return hub;
    }

    MyoDataCollector();// {}
    void init();

    virtual ~MyoDataCollector();
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat) override;
    void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel) override {
        emit AccelerometerData(myo,timestamp,accel);
    }

    void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) override {
        emit GyroscopeData(myo, timestamp, gyro);
    }

    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg) {
        emit EmgData(myo, timestamp, emg);
    }


    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) override;

    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                           myo::WarmupState warmupState) override {
        emit ArmSync(myo, timestamp, arm, xDirection, rotation, warmupState);
    }

    void onArmUnsync(myo::Myo* myo, uint64_t timestamp) override {
        emit ArmUnsync(myo, timestamp);
    }

    void onLock(myo::Myo* myo, uint64_t timestamp) override {
        emit Lock(myo, timestamp);
    }

    void onUnlock(myo::Myo* myo, uint64_t timestamp) override {
        emit Unlock(myo, timestamp);
    }

    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) override {
        emit Connect(myo, timestamp, firmwareVersion);
    }

    void onDisconnect(myo::Myo* myo, uint64_t timestamp) override {
        emit Disconnect(myo, timestamp);
    }   

private:
    // These values are set by onArmRecognized() and onArmLost() above.
    bool onArm;
    myo::Arm whichArm;
    // These values are set by onOrientationData() and onPose() above.
    //int roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
    QTimer hubEventTimer;

signals:
    void Connect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);
    void Disconnect(myo::Myo* myo, uint64_t timestamp);
    void OrientationData(myo::Myo*, uint64_t, const myo::Quaternion<float> quat, float roll, float pitch, float yaw);
    void AccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > accel);
    void GyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > gyro);
    void EmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
    void Pose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);
    void ArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                 myo::WarmupState warmupState);
    void ArmUnsync(myo::Myo* myo, uint64_t timestamp);
    void Lock(myo::Myo* myo, uint64_t timestamp);
    void Unlock(myo::Myo* myo, uint64_t timestamp);

public slots:
    void hubCallback() {getHub().run(interval);}

};

#endif // MYODATACOLLECTOR_H
