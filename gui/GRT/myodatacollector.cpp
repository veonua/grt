#define _USE_MATH_DEFINES
#include <cmath>


#include "myodatacollector.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>


MyoDataCollector::MyoDataCollector() {
    try {
        myo::Myo* myo = getHub().waitForMyo(10000);

        //getHub().setLockingPolicy();
        if (!myo)
            throw std::runtime_error("Unable to find a Myo!");

        getHub().addListener(this);

        hubEventTimer.setSingleShot(false);
        hubEventTimer.setInterval(interval);
        QObject::connect(&hubEventTimer, QTimer::timeout, this, MyoDataCollector::hubCallback);
        hubEventTimer.start(interval);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
    }
}

MyoDataCollector::~MyoDataCollector() {
    getHub().removeListener(this);
}

// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
// as a unit quaternion.
void MyoDataCollector::onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
{    
    using std::atan2;
    using std::asin;
    using std::sqrt;
    using std::max;
    using std::min;

    // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
    float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                       1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
    float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
    float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                    1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

    /* Convert the floating point angles in radians to a scale from 0 to 18.
    roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
    pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
    yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
    */
    emit OrientationData(myo, timestamp, quat, roll, pitch, yaw);
}

// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
// making a fist, or not making a fist anymore.
void MyoDataCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose ppose)
{
    currentPose = ppose;

    if (ppose != myo::Pose::unknown && ppose != myo::Pose::rest) {
        // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
        // Myo becoming locked.
        myo->unlock(myo::Myo::unlockHold);

        // Notify the Myo that the pose has resulted in an action, in this case changing
        // the text on the screen. The Myo will vibrate.
        myo->notifyUserAction();
    } else {
        // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
        // are being performed, but lock after inactivity.
        myo->unlock(myo::Myo::unlockTimed);
    }
    emit Pose(myo, timestamp, ppose);
}


