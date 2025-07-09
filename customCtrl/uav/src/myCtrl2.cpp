#include "myCtrl.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <TabWidget.h>
#include <CheckBox.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>
#include <iostream>
#include <Label.h>
#include <Vector3DSpinBox.h>
#include <Pid.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

void trajectory_gen(float &v, float &x, float &xp, float entrada, float dt, float a) {
    v += dt*a*(entrada - v);
    x += dt*xp;
    xp += dt*(-a*a*x-2*a*xp+a*a*v);
}

MyController::MyController(const LayoutPosition *position, const string &name) : ControlLaw(position->getLayout(),name,4)
{
    first_update = true;

    // Input matrix
    input = new Matrix(this, 4, 6, floatType, name);

    // Matrix descriptor for logging. It should be always a nx1 matrix. 
    MatrixDescriptor *log_labels = new MatrixDescriptor(3, 1);
    log_labels->SetElementName(0, 0, "x_error");
    log_labels->SetElementName(1, 0, "y_error");
    log_labels->SetElementName(2, 0, "yaw_error");
    state = new Matrix(this, log_labels, floatType, name);
    delete log_labels;

    // GUI for custom PID
    GroupBox *gui_customPID = new GroupBox(position, name);
    GroupBox *general_parameters = new GroupBox(gui_customPID->NewRow(), "General parameters");
    deltaT_custom = new DoubleSpinBox(general_parameters->NewRow(), "Custom dt [s]", 0, 1, 0.001, 4);
    mass = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Mass [kg]", 0, 10, 0.01, 4, 0.436);
    k_motor = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Motor constant", 0, 50, 0.01, 4, 29.5870);
    sat_pos = new DoubleSpinBox(general_parameters->NewRow(), "Saturation pos", 0, 10, 0.01, 3);
    sat_att = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation att", 0, 10, 0.01, 3);
    sat_thrust = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation thrust", 0, 10, 0.01, 3);

    // Custom cartesian position controller
    GroupBox *custom_position = new GroupBox(gui_customPID->NewRow(), "Custom position controller");
    Lambdap = new Vector3DSpinBox(custom_position->NewRow(), "Lambda", 0, 100, 0.1, 3);
    KDp = new Vector3DSpinBox(custom_position->LastRowLastCol(), "KD", 0, 100, 0.1, 3);
    Gammap = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Gamma", 0, 1000, 0.1, 3);

    // Custom attitude controller
    GroupBox *custom_attitude = new GroupBox(gui_customPID->NewRow(), "Custom attitude controller");
    Lambdaq = new Vector3DSpinBox(custom_attitude->NewRow(), "Lambda", 0, 100, 0.1, 3);
    KDq = new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "KD", 0, 100, 0.1, 3);
    Gammaq = new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "Gamma", 0, 1000, 0.1, 3);

    AddDataToLog(state);
}

MyController::~MyController()
{
    delete state;
}

void MyController::UpdateFrom(const io_data *data)
{
    float current_time = double(GetTime())/1000000000-initial_time;
    float thrust = 0.0;
    Vector3Df u, tau;

    if(deltaT_custom->Value() == 0)
    {
        delta_t = (float)(data->DataDeltaTime())/1000000000;
    }
    else
    {
        delta_t = deltaT_custom->Value();
    }
   
    if(first_update)
    {
        initial_time = double(GetTime())/1000000000;
        first_update = false;
    }

    // Obtain state
    input->GetMutex();
    Vector3Df pos_error(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    Vector3Df vel_error(input->Value(0, 1), input->Value(1, 1), input->Value(2, 1));
    Quaternion q(input->Value(0, 2), input->Value(1, 2), input->Value(2, 2), input->Value(3, 2));
    Vector3Df omega(input->Value(0, 3), input->Value(1, 3), input->Value(2, 3));
    float yaw_ref = input->Value(0, 4);
    Vector3Df pos_ref(input->Value(0, 5), input->Value(1, 5), input->Value(2, 5));
    input->ReleaseMutex();

    // Get tunning parameters from GUI
    Vector3Df Lambdap_val(Lambdap->Value().x, Lambdap->Value().y, Lambdap->Value().z);
    Vector3Df KDp_val(KDp->Value().x, KDp->Value().y, KDp->Value().z);
    Vector3Df Gammap_val(Gammap->Value().x, Gammap->Value().y, Gammap->Value().z);
    Vector3Df Lambdaq_val(Lambdaq->Value().x, Lambdaq->Value().y, Lambdaq->Value().z);
    Vector3Df KDq_val(KDq->Value().x, KDq->Value().y, KDq->Value().z);
    Vector3Df Gammaq_val(Gammaq->Value().x, Gammaq->Value().y, Gammaq->Value().z);

    Euler rpy = q.ToEuler();
    float a = 100.0f;

    // Position adaptive controller
    // trayectory generator
    trajectory_gen(vpx, xp, xd_p, pos_ref.x, delta_t, a);
    trajectory_gen(vpy, yp, yd_p, pos_ref.y, delta_t, a);
    trajectory_gen(vpz, zp, zd_p, pos_ref.z, delta_t, a);
    float xd_pp = -a*a*xp-2.0f*a*xd_p+a*a*vpx;
    float yd_pp = -a*a*yp-2.0f*a*yd_p+a*a*vpy;
    float zd_pp = -a*a*zp-2.0f*a*zd_p+a*a*vpz;

    // sliding surface
    float xr_p = xd_p - Lambdap_val.x * pos_error.x;
    float yr_p = yd_p - Lambdap_val.y * pos_error.y;
    float zr_p = zd_p - Lambdap_val.z * pos_error.z;

    float xr_pp = xd_pp - Lambdap_val.x * vel_error.x;
    float yr_pp = yd_pp - Lambdap_val.y * vel_error.y;
    float zr_pp = zd_pp - Lambdap_val.z * vel_error.z;

    float spx = vel_error.x + Lambdap_val.x * pos_error.x;
    float spy = vel_error.y + Lambdap_val.y * pos_error.y;
    float spz = vel_error.z + Lambdap_val.z * pos_error.z;

    // adaptive mechanism pi_p = Gamma^(-1)*Y*s
    //pip1 +=  delta_t * (xr_pp*spx+yr_pp*spy+zr_pp*spz)/Gammap_val.x;
    pip1 +=  delta_t * (xr_pp*spx+yr_pp*spy)/Gammap_val.x;
    piz1 +=  delta_t * zr_pp*spz/Gammap_val.z;
    pip2 +=  delta_t * (xr_p*spx+yr_p*spy)/Gammap_val.x;
    pip3 +=  delta_t * zr_p*spz/Gammap_val.z;
    pip4 += -delta_t * spz/Gammap_val.z;

    // control law u = Y*pi + KD*s
    //u.x = pip1*xr_pp + pip2*xr_p + KDp_val.x*spx;
    //u.y = pip1*yr_pp + pip2*yr_p + KDp_val.y*spy;
    //u.z = piz1*zr_pp + pip3*zr_p - pip4 + KDp_val.z*spz;
    u.x = 0.2f*pos_error.x + 0.2f*vel_error.x;
    u.y = 0.2f*pos_error.y + 0.2f*vel_error.y;
    u.z = 0.3f*pos_error.z + 0.1f*vel_error.z;
    std::cout << pos_error.x << "\t" << pos_error.y << "\t" << pos_error.z << std::endl;
    //std::cout << pip1 << "\t" << piz1 << "\t" << pip2 << "\t" << pip3 << "\t" << pip4 << std::endl;

    float ctrl_z = u.z; // This is the thrust needed to control the z position before saturation
    u.Saturate(sat_pos->Value());
    
    // trayectory generator
    trajectory_gen(vx, x, uxd_p, u.x, delta_t, a);
    trajectory_gen(vy, y, uyd_p, u.y, delta_t, a);
    float uxd_pp = -a*a*x-2*a*uxd_p+a*a*vx;
    float uyd_pp = -a*a*y-2*a*uyd_p+a*a*vy;

    // sliding surface
    float uxr_p = - uyd_p + Lambdaq_val.x * (- y - rpy.roll);
    float uyr_p = uxd_p + Lambdaq_val.y * (x - rpy.pitch);
    float uzr_p = Lambdaq_val.z * (0.0f - rpy.yaw);

    float uxr_pp = - uyd_pp + Lambdaq_val.x * (- uyd_p - omega.x);
    float uyr_pp = uxd_pp + Lambdaq_val.y * (uxd_p - omega.y);
    float uzr_pp = Lambdaq_val.z * (0.0f - omega.z);

    float sx = (- uyd_p - omega.x) + Lambdaq_val.x * (- y - rpy.roll);
    float sy = (uxd_p - omega.y) + Lambdaq_val.y * (x - rpy.pitch);
    float sz = (0.0f - omega.z) + Lambdaq_val.z * (0.0f - rpy.yaw);

    // adaptive mechanism pi_p = Gamma^(-1)*Y*s
    pi1 +=  delta_t * uxr_pp*sx/Gammaq_val.x;
    pi2 +=  delta_t * uyr_pp*sy/Gammaq_val.x;
    pi3 +=  delta_t * uzr_pp*sz/Gammaq_val.x;
    pi4 += -delta_t * uyr_p*omega.z*sy/Gammaq_val.x;
    pi5 += -delta_t * uxr_p*omega.z*sx/Gammaq_val.x;

    // control law u = Y*pi + KD*s
    //tau.x = - KDq_val.x*sx + pi1*uxr_pp - pi4*uyr_p*omega.z;
    //tau.y = - KDq_val.y*sy + pi2*uyr_pp - pi5*uxr_p*omega.z;
    //tau.z = - KDq_val.z*sz + pi3*uzr_pp;
    tau.x = 15.0f*(rpy.roll + u.y) + 0.1f*omega.x;
    tau.y = 15.0f*(rpy.pitch - u.x) + 0.1f*omega.y;
    tau.z = 15.0f*(rpy.YawDistanceFrom(yaw_ref)) + 0.1f*omega.z;
    
    //std::cout << rpy.roll + u.y << "\t" << rpy.pitch - u.x << "\t" << pos_error.z << std::endl;
    //std::cout << pi1 << "\t" << pi2 << "\t" << pi3 << "\t" << pi4 << "\t" << pi5 << std::endl;

    applyMotorConstant(tau);
    tau.Saturate(sat_att->Value());

    // Compute custom thrust
    float comp_mg = -mass->Value()*g; // This is the thrust needed to counteract gravity. Based on the default PID, it should be -0.397918 in Fl-Air simulator.  
    thrust = ctrl_z+comp_mg; // This is the thrust needed to counteract gravity and control the z position
    applyMotorConstant(thrust);
    if(thrust < -sat_att->Value())
    {
        thrust = -sat_att->Value();
    }
    else if(thrust >= 0.0f)
    {
        thrust = 0.0f; 
    }
    // Debug thrust value
    //std::cout << thrust << std::endl;
    // std::cout << "Thrust comp: " << comp_mg/k_motor->Value() << " ctrl_z: " << ctrl_z << " thrust: " << thrust << "z-error: " << pos_error.z << std::endl;

    // Send controller output
    output->SetValue(0, 0, tau.x);
    output->SetValue(1, 0, tau.y);
    output->SetValue(2, 0, tau.z);
    output->SetValue(3, 0, thrust);
    output->SetDataTime(data->DataTime());

    // Log state (example). 
    // Modify the log_labels matrix in the constructor to add more variables.
    state->GetMutex();
    state->SetValue(0, 0, pos_error.x);
    state->SetValue(1, 0, pos_error.y);
    state->SetValue(2, 0, rpy.YawDistanceFrom(0));
    state->ReleaseMutex();

    ProcessUpdate(output);
}

void MyController::Reset(void)
{
    first_update = true;
}

void MyController::SetValues(Vector3Df pos_error, Vector3Df vel_error, Quaternion currentQuaternion, Vector3Df omega, float yaw_ref, Vector3Df pos_ref)
{
    // Set the input values for the controller. 
    // This function is called from the main controller to set the input values.
    input->GetMutex();
    input->SetValue(0, 0, pos_error.x);
    input->SetValue(1, 0, pos_error.y);
    input->SetValue(2, 0, pos_error.z);

    input->SetValue(0, 1, vel_error.x);
    input->SetValue(1, 1, vel_error.y);
    input->SetValue(2, 1, vel_error.z);

    input->SetValue(0, 2, currentQuaternion.q0);
    input->SetValue(1, 2, currentQuaternion.q1);
    input->SetValue(2, 2, currentQuaternion.q2);
    input->SetValue(3, 2, currentQuaternion.q3);

    input->SetValue(0, 3, omega.x);
    input->SetValue(1, 3, omega.y);
    input->SetValue(2, 3, omega.z);

    // Set yaw reference
    input->SetValue(0, 4, yaw_ref);

    input->SetValue(0, 5, pos_ref.x);
    input->SetValue(1, 5, pos_ref.y);
    input->SetValue(2, 5, pos_ref.z);

    input->ReleaseMutex();
}

void MyController::applyMotorConstant(Vector3Df &signal)
{
    float motor_constant = k_motor->Value();
    signal.x = signal.x/motor_constant;
    signal.y = signal.y/motor_constant;
    signal.z = signal.z/motor_constant;
}

void MyController::applyMotorConstant(float &signal)
{
    float motor_constant = k_motor->Value();
    signal = signal/motor_constant;
}