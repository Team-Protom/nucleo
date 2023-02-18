#include <dcmotorPID.hpp>

namespace hardware
{

    namespace drivers
    { 

        namespace PID {
            //This function initializes PID
            void init_PID(dcmotorPID* p, float tc, float u_max, float u_min)
            {
                // set PID parameters
                p->tc = tc; // sampling period
                p->u_max = u_max; // PID UOTPUT UPPER LIMIT
                p->u_min = u_min; // PID UOTPUT LOWER LIMIT
            }

            void tune_PID(dcmotorPID* p, float kp, float ki, float kd)
            {
                p->kp = kp;
                p->ki = ki;
                p->kd = kd;
            }

            float pid_controller(dcmotorPID* p, float y, float r)
            {
                static float e_old = 0, i_term = 0;
                float u;

                float e = r - y;

                float p_term =p->kp * e;
                float new_i_term = i_term + (p->ki)*(p->tc)*e_old;
                float d_term = (p->kd/p->tc)*(e - e_old);

                e_old = e;

                u = p_term + new_i_term + d_term;

                if (u > p->u_max){
                    u = p->u_max;
                }
                else if (u < p->u_min){
                    u = p->u_min;
                }
                else {
                    i_term = new_i_term;
                }

                return u;
            }

        }
    }
}