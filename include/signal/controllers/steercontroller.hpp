/**
******************************************************************************
* @file    steercontroller.hpp
* @author  RBRO/PJ-IU
* @version V1.0.0
* @date    2023-02-20
* @brief   
******************************************************************************
*/
#ifndef STEERCONTROLLER_H
#define STEERCONTROLLER_H
#include "steercontroller.inl"
#include <signal/controllers/sisocontrollers.hpp>

#include <mbed.h>

namespace signal
{
    namespace controllers
    {

   /**
    * @brief It implements a controller with a single input and a single output. It can be completed with a converter to convert the measaurment unit of the control signal. 
    * 
    */
        class SteerController
        {
            /* PID controller declaration*/
            template <class T>
            using  ControllerType=siso::IController<T>;

            public:
                /* Construnctor */
                SteerController(ControllerType<double>&              f_pid
                                ,float                                  f_inf_pwm
                                ,float                                  f_sup_pwm);
                /* Set controller reference value */
                void setRef(double    f_RefRps);
                /* Set actual value */
                void setYaw(double    f_yaw);
                /* Get value */
                double getYaw();
                /* Get control value */
                double getAngle();
                /* Clear PID parameters */
                void clearAngle();
                /* Control action */
                int8_t control();
                /* Converts to pwm */
                float conversion(float f_angle);

                bool inRange(double f_RefRps);
                bool pwmInRange(double f_RefDist);

            private:
                /* PID object reference */
                ControllerType<double>&                 s_pid;
                /* Controller reference */
                double                                  m_yawRef;
                /* Distance reference */
                double                                  m_RefDist;
                /* Control value */
                double                                  yaw_rate;
                /* Control value */
                float                                   pwm_value;


                /* Scaled PWM control signal limits */
                const float                                     m_control_sup;
                const float                                     m_control_inf;
                /* PWM Signal allowed limits */
                const float                                     m_inf_pwm;
                const float                                     m_sup_pwm;

        };
    }; // namespace controllers

}; // namespace signal

#endif // STEERCONTROLLER_H
