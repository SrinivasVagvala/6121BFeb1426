#ifndef _INTAKE_HPP_
#define _INTAKE_HPP_






void intakeOpControl();
void heatSensingControl();
void matchLoadSorting();

void intakeTask(void* parameter);

void setMatch(bool state);
void setTeam(bool state);

void intake(bool state); // done
void extake(bool state); // done
void reverseScoring(bool state); // done

void scoreHigh(bool state); // done
void scoreMid(bool state, bool useExtake); // done
void scoreLow(bool state); // done


void MiddleGoalScoreSkills(bool state); // done

void turnOffLower(bool state); // done
void turnOffScoring(bool state); // done
void turnLoweratXSpeed(bool state,int speed); // done
void turnScoringatXSpeed(bool state, int speed); // done


void antiJam(bool direction);

void setAntiJam(bool state, bool antiS);








#endif