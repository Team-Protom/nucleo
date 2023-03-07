#include <hardware/sensors/sonar.hpp>>

namespace hardware::sensors {
    sonar::sonar(RawSerial& f_serialPort, PinName trigPin, PinName echoPin, float update_Speed, float time_out)
    :triggerP(trigPin)
    ,echoP(echoPin)
    ,m_serialPort(f_serialPort)
    {
        updateSpeed = update_Speed;
        timeout = time_out;
        echoP.rise(this,&sonar::starT);
        echoP.fall(this,&sonar::updateDist);
        echoP.enable_irq();
    };


    void sonar::starT()
    {
        if(timer.read()>600)
        {
            timer.reset ();
        }
        start = timer.read_us();
    };

    void sonar::startTimer()
    {
        timer.start(); // start the timer
    };
        
    void sonar::updateDist()
    {
        end = timer.read_us();
        done = 1;
        distance = (end - start)*0.03431/2;       
        tout.detach();
        tout.attach(this,&sonar::trigger, updateSpeed);   
    };

    void sonar::trigger(void)
    {
            tout.detach();
            triggerP = 1;             
            wait_us(10);   
            triggerP = 0;
            tout.attach(this,&sonar::trigger, timeout); 
    };
    
    float sonar::getCurrentDistance(void)
    {
        return distance;
    };

    void sonar::pauseUpdates(void)
    {
        tout.detach();
        echoP.rise(NULL);
        echoP.fall(NULL);
    };

    void sonar::startUpdates(void)
    {
        trigger();
    };

    void sonar::changeUpdateSpeed(float update_Speed)
    {
        updateSpeed = update_Speed;
    };

    float sonar::getUpdateSpeed()
    {
        return updateSpeed;
    };

    /** \brief  Serial callback method for speed command
     *
     * Serial callback method for getting the distance from the sonar
     *
     * @param a                   string to read data 
     * @param b                   string to write data 
     * 
     */
    void sonar::serialcallbackDISTANCEcommand(char const * a, char * b)
    {
        float distance, temp;
        uint32_t l_res = sscanf(a, "%f", &temp);
        if(0 == l_res)
        {
            distance = sonar::getCurrentDistance();
            m_serialPort.printf("@5:%5f;;\r\n", distance);
        }
        else
        {
            sprintf(b,"sintax error;;");
        }
    };


}
