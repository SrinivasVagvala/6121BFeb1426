#ifndef _INTAKE_HPP_
#define _INTAKE_HPP_






void intakeOpControl();
void heatSensingControl();
void matchLoadSorting();

void intakeTask(void* parameter);

void setMatch(bool state);
void setTeam(bool state);

void intake(bool state);
void extake(bool state);
void reverseScoring(bool state);

void scoreHigh(bool state);
void scoreMid(bool state, bool useExtake);
void scoreLow(bool state);

void MiddleAlignerActive(bool state);
void MiddleAlignerInactive(bool state);

void MiddleGoalScoreSkills(bool state);

void turnOffLower(bool state);
void turnOffScoring(bool state);
void turnLoweratXSpeed(bool state,int speed);
void turnScoringatXSpeed(bool state, int speed);


void antiJam();

void setAntiJam(bool state);

void descoreMidToggle();








#endif