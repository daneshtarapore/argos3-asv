/*
 * AUTHOR: Danesh Tarapore  <daneshtarapore@gmail.com>
 *
 * An example controller for the ASV bot.
 *
 * This controller is very simple and merely makes the robot go straight. 
 * The controller uses the dual propellers  move the robot.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/asv_1.argos
 */

#ifndef ASV_TEST_H
#define ASV_TEST_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/asv/control_interface/ci_asv_dualpropeller_actuator.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CASVTest : public CCI_Controller {

public:

   /* Class constructor. */
   CASVTest();

   /* Class destructor. */
   virtual ~CASVTest() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><asv_test_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

private:

   /* Pointer to the differential steering actuator */
   CCI_ASVDualPropellerActuator* m_pcPropellers;


   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><asv_test_controller> section.
    */

   /* Propeller thrust force. */
   Real m_fThrustForce;


   Real m_fSimulationClockTick;


};

#endif
