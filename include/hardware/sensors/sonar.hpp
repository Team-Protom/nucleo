/**
******************************************************************************
* @file    sonar.hpp
* @author  RBRO/PJ-IU
* @version V1.0.0
* @date    2023-02-18
* @brief   
******************************************************************************
*/
#include <mbed.h>

#ifndef MBED_ULTRASONIC_H
#define MBED_ULTRASONIC_H
namespace hardware::sensors {
    class sonar
    {
        public:
            /**iniates the class with the specified trigger pin, echo pin, update speed and timeout**/
            sonar(RawSerial& f_serialPort, PinName trigPin, PinName echoPin, float update_Speed, float time_out);
            /** returns the last measured distance**/
            float getCurrentDistance(void);
            /**pauses measuring the distance**/
            void pauseUpdates(void);
            /**starts mesuring the distance**/
            void startUpdates(void);
            /**changes the speed at which updates are made**/
            void changeUpdateSpeed(float updateSpeed);
            /**gets the speed at which updates are made**/
            float getUpdateSpeed(void);
            void startTimer();
            /* Serial callback for getting the distance */
            void serialcallbackDISTANCEcommand(char const * a, char * b);

            
        private:
            /* reference to Serial object */
            RawSerial& m_serialPort;

            DigitalOut triggerP;
            InterruptIn echoP;
            Timer timer;
            Timeout tout;
            float distance;
            float updateSpeed;
            int start;
            int end;
            volatile int done;
            void (*_onUpdateMethod)(int);
            void starT(void);
            void updateDist(void);
            void trigger();
            float timeout;
            int d;
    };

}

#endif

