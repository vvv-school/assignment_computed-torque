#ifndef MODULE_H
#define MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>

#include <iDynTree/KinDynComputations.h>

class Module : public yarp::os::RFModule
{
    // Class to compute model quantities 
    iDynTree::KinDynComputations kinDynModel;
    
    // Attributes to communicate with the robot YARP-based interface 
    yarp::dev::PolyDriver robotDevice;
    
    // YARP Interfaces exposed by the remotecontrolboardremapper 
    yarp::dev::IControlLimits2   *ilim{nullptr};
    yarp::dev::IEncoders         *ienc{nullptr};
    yarp::dev::IControlMode2     *imod{nullptr};
    yarp::dev::ITorqueControl    *itrq{nullptr};

public:
    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};


#endif /* end of include guard: MODULE_H */
