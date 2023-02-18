#include <mbed.h>
#include <rtos.h>


class HCSR04 {
    
    public:
    
    /** Receives two PinName variables.
     * @param echoPin mbed pin to which the echo signal is connected to
     * @param triggerPin mbed pin to which the trigger signal is connected to
     */
    HCSR04(PinName echoPin, PinName triggerPin);

    /* Distruttore */
    virtual ~HCSR04();

    
    /** Calculates the distance in cm, with the calculation time of 25 ms.
     * @returns distance of the measuring object in cm.
     */
    float getDistance_cm();
    
    private:
    
    InterruptIn echo;       // echo pin
    DigitalOut trigger;     // trigger pin
    Timer timer;            // echo pulsewidth measurement
    float distance;         // store the distance in cm
    
    /** Start the timer. */
    void startTimer();
    
    /** Stop the timer. */
    void stopTimer();
    
    /** Initialization. */
    void init();
    
    /** Start the measurement. */
    void startMeasurement();
};