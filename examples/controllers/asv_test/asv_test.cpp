/* Include the controller definition */
#include "asv_test.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/core/simulator/simulator.h>

/****************************************/
/****************************************/

CASVTest::CASVTest() :
   m_pcPropellers(NULL),
   m_fThrustForce(2.5f),
   m_fSimulationClockTick(0.0f)
{}

/****************************************/
/****************************************/

void CASVTest::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "dual_propeller") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><asv_test><actuators> and
    * <controllers><asv_test><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcPropellers    = GetActuator<CCI_ASVDualPropellerActuator>("dual_propeller");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */



}

/****************************************/
/****************************************/

void CASVTest::ControlStep()
{
    std::cout << m_fSimulationClockTick << std::endl;

    if((this->GetId().compare("asv_1") == 0) && (m_fSimulationClockTick < 10.0f))
    {
        /* Go straight */
        m_pcPropellers->SetPropellerThrust(13.0f, 13.0f);
        m_pcPropellers->SetPropellerAngle(0);
    }
    else
    {
        m_pcPropellers->SetPropellerThrust(0.0f, 0.0f);
        m_pcPropellers->SetPropellerAngle(0);
    }

    m_fSimulationClockTick++;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CASVTest, "asv_test_controller")
