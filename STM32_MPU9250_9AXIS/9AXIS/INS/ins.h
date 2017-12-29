#ifndef _INS_H_
#define _INS_H_

typedef struct
{
  float InsPos;
  float InsVel;
  float accZnog;
  float accZnogoffset;
  double accZnogSum;
}INERTIAL_TYPEDEF;

typedef struct
{
  INERTIAL_TYPEDEF X;
  INERTIAL_TYPEDEF Y;
  INERTIAL_TYPEDEF Z;
  unsigned short runcnt;
  unsigned short calflag;
}INS_TYPEDEF;

void INS_Update(float dt);

#endif
