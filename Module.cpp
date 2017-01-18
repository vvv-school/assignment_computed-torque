#include <Module.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

double Module::getPeriod () { return 0.01; }

bool Module::updateModule ()
{
    // Implement the Computed Torque controller
    
    return true;
}

bool Module::configure (yarp::os::ResourceFinder &rf)
{
    using namespace yarp::os;

    Property wbiProperties;
    std::string wbiConfFile = rf.check("wbi_config_file", Value("yarpWholeBodyInterface.ini"), "Checking wbi configuration file").asString();

    if (!wbiProperties.fromConfigFile(rf.findFile(wbiConfFile))) {
        std::cout << "Not possible to load WBI properties from file.\n";
        return false;
    }
    wbiProperties.fromString(rf.toString(), false);

    //retrieve the joint list
    std::string wbiList = rf.check("wbi_list", Value("ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP"), "Looking for wbi list").asString();

    wbi::IDList iCubMainJoints;
    if (!yarpWbi::loadIdListFromConfig(wbiList, wbiProperties, iCubMainJoints)) {
        std::cout << "Cannot find joint list\n";
        return false;
    }

    unsigned actuatedDOFs = iCubMainJoints.size();

    //create an instance of wbi
    m_robot = new yarpWbi::yarpWholeBodyInterface("computed_torque", wbiProperties);
    if (!m_robot) {
        std::cout << "Could not create wbi object.\n";
        return false;
    }

    m_robot->addJoints(iCubMainJoints);
    if (!m_robot->init()) {
        std::cout << "Could not initialize wbi object.\n";
        return false;
    }
    
    //Creating and configuring a wbi instance.
    //We use the yarp implementation.

    //Load information about the joints and global properties for the wbi
    //this file also contains references to the urdf model
    
    std::cout << "Number of DOFs: " << actuatedDOFs << "\n";

    //Any further initialization
    
    
    return true;
}

bool Module::close ()
{

    //cleanup stuff
    m_robot->close();
    delete m_robot;
    m_robot = 0;
    return true;
}

