#ifndef _ARM_PWM_H
#define _ARM_PWM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>

const std::string SYSFS_PWM_DIR = "/sys/devices/ocp.3";

int setDuty(int duty, std::string device);
int getDuty(std::string device);
int getPeriode(std::string device);

#endif
