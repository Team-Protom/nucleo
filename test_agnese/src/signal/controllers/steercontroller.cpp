#include <signal/controllers/steercontroller.hpp>

namespace signal{
    

namespace controllers{
    /**
     * @brief Construct a new CMotorController::CMotorController object
     * 
     * @param f_pid     Reference to the controller interface.
     * @param f_inf_pwm   [Optional] Inferior limit of pwm signal.
     * @param f_sup_pwm   [Optional] Superior limit of pwm signal.
     */
    SteerController::SteerController(ControllerType<double>&                  f_pid
                            ,float                                                  f_inf_pwm
                            ,float                                                  f_sup_pwm)
        :s_pid(f_pid)
        ,m_control_sup(30)
        ,m_control_inf(-30)
        ,m_inf_pwm(f_inf_pwm)
        ,m_sup_pwm(f_sup_pwm)
    {
    };

    /**
     * @brief Set the reference signal value.
     * 
     * @param f_RefRps The value of the reference signal
     */
    void SteerController::setRef(double yaw_ref)
    {
        m_yawRef=yaw_ref;
    };

    /** @brief  Get the value of the control signal calculated last time.
     * 
     */
    double SteerController::getYaw()
    {
        return m_yawRef;
    };

    /** @brief  Get the control value.
     * 
     */
    double SteerController::getAngle()
    {
        return pwm_value;
    };

    /** @brief  Clear the memory of the controller. 
     *
     */
    void SteerController::clearAngle()
    {
        s_pid.clear();
    };

    /**
     * @brief It calculates the next value of the control signal, by utilizing the given interfaces.
     * 
     * @return true control works fine
     * @return false appeared an error
     */
    int8_t SteerController::control()
    {
        float l_ref;
        float l_error = m_yawRef - yaw_rate;

        float l_v_control = s_pid.calculateControl(l_error);
        float l_pwm_control = conversion(l_v_control);
        
        // Verify that the pwm value is in the limits
        if(l_pwm_control<m_inf_pwm)
        {
            pwm_value = m_inf_pwm;
        }
        else 
        {
            if (l_pwm_control>m_sup_pwm)
            {
                pwm_value = m_sup_pwm;
            }
            else
            {
                pwm_value = l_pwm_control;
            }
        }

        return 1;
    };

    /* Converts to duty cycle for pwm signal. 
     * 
     *  @param f_angle    angle degree
     *  \return         duty cycle in interval [0,1]
     */
    float SteerController::conversion(float f_angle)
    {
        return (0.0009505 * f_angle + 0.07525);
    };

    void SteerController::setYaw(double f_yaw)
    {
        yaw_rate = f_yaw;
    };

    /**
     * @brief It verifies whether a number is in a given range
     * 
     * @param f_RefRps reference value for controller in rotation per second  
     * @return true means, that the value is in the range
     * @return false means, that the value isn't in the range
     */
    bool SteerController::inRange(double f_RefRps){
        return m_inf_pwm<=f_RefRps && f_RefRps<=m_sup_pwm;
    };

    /**
     * @brief It checks whether a number is in a given range
     * 
     * @param f_RefDist value 
     * @return true means, that the value is in the range
     * @return false means, that the value isn't in the range
     */
    bool SteerController::pwmInRange(double f_RefDist){
        return m_inf_pwm<=f_RefDist && f_RefDist <=m_sup_pwm;
    };

}; //  namespace controllers
};// namespace signal
