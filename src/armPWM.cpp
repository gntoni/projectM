#include "armPWM.h"

int setDuty(int duty, std::string device)
{
  std::stringstream filename;
  filename << SYSFS_PWM_DIR << "/" << device << "/duty";
  std::ofstream pwmFile(filename.str().c_str());
  if (pwmFile.is_open())
  {
    pwmFile << duty;
    pwmFile.close();
    return 0;
  }
  else
  {
    std::cout << "Unable to write to file " << device << std::endl;
    return -1;
  }
}

int getDuty(std::string device)
{
  return -1;
}

int getPeriode(std::string device)
{
  return -1;
}

int main(int argc,  char* argv[])
{
  std::string pwm1 = "pwm1_inmoov.15";
  std::string pwm2 = "pwm2_inmoov.16";
  std::string pwm3 = "pwm3_inmoov.17";
  std::string pwm4 = "pwm4_inmoov.18";
  int duty1 = 19500000;
  int duty2 = 18000000;

  while(1)
  {
    setDuty(duty1, pwm1);
    setDuty(duty1, pwm2);
    setDuty(duty1, pwm3);
    setDuty(duty1, pwm4);
    sleep(3);
    setDuty(duty2, pwm1);
    setDuty(duty2, pwm2);
    setDuty(duty2, pwm3);
    setDuty(duty2, pwm4);
    sleep(3);
  }
}

