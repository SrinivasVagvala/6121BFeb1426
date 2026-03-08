// #include "main.h"
// #include "quadsensor.hpp"


// // RobotPos rohan;



// pros::Distance frontD(FRONTDISTANCE);
// pros::Distance backD(BACKDISTANCE);
// pros::Distance leftD(LEFTDISTANCE);
// pros::Distance rightD(RIGHTDISTANCE);




// void posTask(void* parameter) {



//     while (true) {

//         if (!pros::competition::is_autonomous()) {

//             master.print(0,0,"Front: %i",frontD.get());

//             posOpControl();

//         }
         
//         pros::delay(10); 
//     }
// }



// void posOpControl() {
// }



// void distanceCorrection(int targetD, int distFromWall, int speed, SensorSide side) {

//     chassis.pid_drive_set(targetD, speed, true);
//     chassis.pid_wait();

//     if (side == FRONT) {
//         float dtm = frontD.get() - distFromWall;

//         chassis.pid_drive_set(dtm, speed, true);
//         chassis.pid_wait();
//     }
//     else if (side == BACK) {
//         float dtm = backD.get() - distFromWall;

//         chassis.pid_drive_set(dtm, speed, true);
//         chassis.pid_wait();
//     } 

// }

// void quadSensorReset() {
//     //chassis.odom_xyt_set((double)rohan.X[0], (double)rohan.X[1], (double)rohan.X[2]);
// }