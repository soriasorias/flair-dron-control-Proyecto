#ifndef MYCTRL_H
#define MYCTRL_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Quaternion.h>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
        class Vector3DSpinBox;
    }
    namespace filter {
        // If you prefer to use a custom controller class, you can define it here.
        // ...
    }
}

namespace flair {
    namespace filter {
        class MyController : public ControlLaw
        {
            public :
                MyController(const flair::gui::LayoutPosition *position, const std::string &name);
                ~MyController();
                void UpdateFrom(const flair::core::io_data *data);
                void Reset(void);
                void SetValues(flair::core::Vector3Df pos_error, flair::core::Vector3Df vel_error, flair::core::Quaternion currentQuaternion, flair::core::Vector3Df omega, float yaw_ref, flair::core::Vector3Df pos_ref);
                void applyMotorConstant(flair::core::Vector3Df &signal);
                void applyMotorConstant(float &signal);

            private : 
                float delta_t, initial_time;
                float g = 9.81;
                bool first_update;
                float pip1 = 0.0f;
                float piz1 = 0.0f;
                float pip2 = 0.0f;
                float pip3 = 0.0f;
                float pip4 = 0.0f;
                float pi1 = 0.0f;
                float pi2 = 0.0f;
                float pi3 = 0.0f;
                float pi4 = 0.0f;
                float pi5 = 0.0f;
                // attitude
                float vx = 0.0f;
                float vy = 0.0f;
                float x = 0.0f;
                float y = 0.0f;
                float uxd_p = 0.0f;
                float uyd_p = 0.0f;
                // position
                float vpx = 0.0f;
                float vpy = 0.0f;
                float vpz = 0.0f;
                float xp = 0.0f;
                float yp = 0.0f;
                float zp = 0.0f;
                float xd_p = 0.0f;
                float yd_p = 0.0f;
                float zd_p = 0.0f;

                flair::core::Matrix *state;
                //flair::gui::Vector3DSpinBox *Kp_pos, *Kd_pos, *Ki_pos, *Kp_att, *Kd_att, *Ki_att;
                flair::gui::Vector3DSpinBox *Lambdap, *KDp, *Gammap, *Lambdaq, *KDq, *Gammaq;
                flair::gui::DoubleSpinBox *deltaT_custom, *mass, *k_motor, *sat_pos, *sat_att, *sat_thrust;
        };
    }
}

#endif // MYCTRL_H