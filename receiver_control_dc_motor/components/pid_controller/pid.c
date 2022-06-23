#include "pid.h"

// void pid_create(pid_controller_t *pid, uint32_t sample_time, float *in, float *out, float *set_point, float kp, float ki, float kd)
// {
//     pid->input = in;
//     pid->output = out;
//     pid->set_point = set_point;
//     pid->auto_mode = false;

//     pid_limits(pid, 0, 3000);
//     pid->sample_time = sample_time;

//     pid_direction(pid, E_PID_DIRECT);
//     pid_controller_tune(pid, kp, ki, kd);

//     pid->last_time = xTaskGetTickCount() * portTICK_PERIOD_MS - pid->sample_time;
// }


// bool pid_need_compute(pid_controller_t *pid)
// {
//     // Check if the PID period has elapsed
//     return (xTaskGetTickCount() * portTICK_PERIOD_MS - pid->last_time >= pid->sampletime) ? true : false;
// }

// void pid_compute(pid_controller_t *pid)
// {
//     // Check if control is enabled
//     if (!pid->automode)
//         return false;

//     float in = *(pid->input);
//     // Compute error
//     float error = (*(pid->setpoint)) - in;
//     // Compute integral
//     pid->iterm += (pid->Ki * error);
//     if (pid->iterm > pid->omax)
//         pid->iterm = pid->omax;
//     else if (pid->iterm < pid->omin)
//         pid->iterm = pid->omin;
//     // Compute differential on input
//     float dinput = in - pid->lastin;
//     // Compute PID output
//     float out = pid->Kp * error + pid->iterm - pid->Kd * dinput;
//     // Apply limit to output value
//     if (out > pid->omax)
//         out = pid->omax;
//     else if (out < pid->omin)
//         out = pid->omin;
//     // Output to pointed variable
//     (*pid->output) = out;
//     // Keep track of some variables for next execution
//     pid->lastin = in;
//     pid->last_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
// }

// void pid_controller_tune(pid_controller_t *pid, float kp, float ki, float kd)
// {
//     // Check for validity
//     if (kp < 0 || ki < 0 || kd < 0)
//         return;

//     // Compute sample time in seconds
//     float ssec = ((float)pid->sampletime) / ((float)TICK_SECOND);

//     pid->Kp = kp;
//     pid->Ki = ki * ssec;
//     pid->Kd = kd / ssec;

//     if (pid->direction == E_PID_REVERSE)
//     {
//         pid->Kp = 0 - pid->Kp;
//         pid->Ki = 0 - pid->Ki;
//         pid->Kd = 0 - pid->Kd;
//     }
// }

// void pid_sample(pid_controller_t *pid, uint32_t time)
// {
//     if (time > 0)
//     {
//         float ratio = (float)(time * (TICK_SECOND / 1000)) / (float)pid->sampletime;
//         pid->Ki *= ratio;
//         pid->Kd /= ratio;
//         pid->sampletime = time * (TICK_SECOND / 1000);
//     }
// }

// void pid_limits(pid_controller_t *pid, float min, float max)
// {
//     if (min >= max)
//         return;
//     pid->omin = min;
//     pid->omax = max;
//     // Adjust output to new limits
//     if (pid->auto_mode)
//     {
//         if (*(pid->output) > pid->omax)
//             *(pid->output) = pid->omax;
//         else if (*(pid->output) < pid->omin)
//             *(pid->output) = pid->omin;

//         if (pid->iterm > pid->omax)
//             pid->iterm = pid->omax;
//         else if (pid->iterm < pid->omin)
//             pid->iterm = pid->omin;
//     }
// }

// void pid_auto(pid_controller_t *pid)
// {
//     // If going from manual to auto
//     if (!pid->auto_mode)
//     {
//         pid->iterm = *(pid->output);
//         pid->lastin = *(pid->input);
//         if (pid->iterm > pid->omax)
//             pid->iterm = pid->omax;
//         else if (pid->iterm < pid->omin)
//             pid->iterm = pid->omin;
//         pid->auto_mode = true;
//     }
// }

// void pid_manual(pid_controller_t *pid)
// {
//     pid->auto_mode = false;
// }

// void pid_direction(pid_controller_t *pid, enum pid_control_directions dir)
// {
//     if (pid->auto_mode && pid->direction != dir)
//     {
//         pid->Kp = (0 - pid->Kp);
//         pid->Ki = (0 - pid->Ki);
//         pid->Kd = (0 - pid->Kd);
//     }
//     pid->direction = dir;
// }