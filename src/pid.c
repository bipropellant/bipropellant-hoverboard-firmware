/*	Floating point PID control loop for Microcontrollers
	Copyright (C) 2015 Jesus Ruben Santa Anna Zamudio.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Author website: http://www.geekfactory.mx
	Author e-mail: ruben at geekfactory dot mx
 */
#include "pid.h"

p_pid_controller pid_create(p_pid_controller pid, float* in, float* out, float* set, float kp, float ki, float kd)
{
	pid->input = in;
	pid->output = out;
	pid->setpoint = set;
	pid->automode = false;

	pid_limits(pid, 0, 255);

	// Set default sample time to 100 ms
	pid->sampletime = 100 * (TICK_SECOND / 1000);

	pid_direction(pid, E_PID_DIRECT);
	pid_tune(pid, kp, ki, kd);

	pid->lasttime = tick_get() - pid->sampletime;

	return pid;
}

bool pid_need_compute(p_pid_controller pid)
{
	// Check if the PID period has elapsed
	return(tick_get() - pid->lasttime >= pid->sampletime) ? true : false;
}

void pid_compute(p_pid_controller pid)
{
	// Check if control is enabled
	if (!pid->automode)
		return;
	
	float in = *(pid->input);
	// Compute error
	float error = (*(pid->setpoint)) - in;
	// Compute integral
	pid->iterm += (pid->Ki * error);
	if (pid->iterm > pid->omax)
		pid->iterm = pid->omax;
	else if (pid->iterm < pid->omin)
		pid->iterm = pid->omin;
	// Compute differential on input
	float dinput = in - pid->lastin;
	// Compute PID output
	float out = pid->Kp * error + pid->iterm - pid->Kd * dinput;
	// Apply limit to output value
	if (out > pid->omax)
		out = pid->omax;
	else if (out < pid->omin)
		out = pid->omin;
	// Output to pointed variable
	(*pid->output) = out;
	// Keep track of some variables for next execution
	pid->lastin = in;
	pid->lasttime = tick_get();;
}

void pid_tune(p_pid_controller pid, float kp, float ki, float kd)
{
	// Check for validity
	if (kp < 0 || ki < 0 || kd < 0)
		return;
	
	//Compute sample time in seconds
	float ssec = ((float) pid->sampletime) / ((float) TICK_SECOND);

	pid->Kp = kp;
	pid->Ki = ki * ssec;
	pid->Kd = kd / ssec;

	if (pid->direction == E_PID_REVERSE) {
		pid->Kp = 0 - pid->Kp;
		pid->Ki = 0 - pid->Ki;
		pid->Kd = 0 - pid->Kd;
	}
}

void pid_sample(p_pid_controller pid, uint32_t time)
{
	if (time > 0) {
		float ratio = (float) (time * (TICK_SECOND / 1000)) / (float) pid->sampletime;
		pid->Ki *= ratio;
		pid->Kd /= ratio;
		pid->sampletime = time * (TICK_SECOND / 1000);
	}
}

void pid_limits(p_pid_controller pid, float min, float max)
{
	if (min >= max) return;
	pid->omin = min;
	pid->omax = max;
	//Adjust output to new limits
	if (pid->automode) {
		if (*(pid->output) > pid->omax)
			*(pid->output) = pid->omax;
		else if (*(pid->output) < pid->omin)
			*(pid->output) = pid->omin;

		if (pid->iterm > pid->omax)
			pid->iterm = pid->omax;
		else if (pid->iterm < pid->omin)
			pid->iterm = pid->omin;
	}
}

void pid_auto(p_pid_controller pid)
{
	// If going from manual to auto
	if (!pid->automode) {
		pid->iterm = *(pid->output);
		pid->lastin = *(pid->input);
		if (pid->iterm > pid->omax)
			pid->iterm = pid->omax;
		else if (pid->iterm < pid->omin)
			pid->iterm = pid->omin;
		pid->automode = true;
	}
}

void pid_manual(p_pid_controller pid)
{
	pid->automode = false;
}

void pid_direction(p_pid_controller pid, enum pid_control_directions dir)
{
	if (pid->automode && pid->direction != dir) {
		pid->Kp = (0 - pid->Kp);
		pid->Ki = (0 - pid->Ki);
		pid->Kd = (0 - pid->Kd);
	}
	pid->direction = dir;
}