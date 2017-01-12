#ifndef MODULE_H
#define MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

namespace wbi {
    class wholeBodyInterface;
}

class Module : public yarp::os::RFModule
{
    wbi::wholeBodyInterface* m_robot;

public:
    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};


#endif /* end of include guard: MODULE_H */
