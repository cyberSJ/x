#include <iostream>
#include <string>
#include <math.h>

using namespace std;

int main(int argc, char** argv){
  float belOpen = 0.5;
  float belClosed = 0.5;

  float pSOpenIsOpen = 0.6;
  float pSClosedIsOpen = 1 - pSOpenIsOpen;
  float pSOpenIsClosed = 0.2;
  float pSClosedIsClosed = 1 - pSOpenIsClosed;

  float pIsOpenPushIsOpen = 1;
  float pIsClosedPushIsOpen = 1 - pIsOpenPushIsOpen;
  float pIsOpenPushIsClosed = 0.8;
  float pIsClosedPushIsClosed = 1 - pIsOpenPushIsClosed;

  float pIsOpenNotIsOpen = 1;
  float pIsClosedNotIsOpen = 1 - pIsOpenNotIsOpen;
  float pIsOpenNotIsClosed = 0;
  float pIsClosedNotIsClosed = 1 - pIsOpenNotIsClosed;

  // Initialize
  string action;
  float belBarOpen = 0;
  float belBarClosed = 0;
  string sensor;
  float norm; 	// Normalizer constant calculated from the first calculation of belOpen and belClosed

  // Main Baye's Filter loop
  for (int t = 1; t <= 4; t++){
    cout << "What's the robot's action? (p / n) : ";
    cin >> action;

    // Calculate belBarOpen and belBarClosed
    if (action == "p"){
      cout << "Robot is pushing\n";

      // Calculating belBar for the door being open through this action...
      belBarOpen = pIsOpenPushIsOpen*belOpen + pIsOpenPushIsClosed*belClosed;
      
      // Calculating belBar for the door being closed through this action...
      belBarClosed = pIsClosedPushIsOpen*belOpen + pIsClosedPushIsClosed*belClosed;
    }
    else if(action == "n"){
      cout << "Robot is doing nothing\n";
      belBarOpen = pIsOpenNotIsOpen*belOpen + pIsOpenNotIsClosed*belClosed;
      belBarClosed = pIsClosedNotIsOpen*belOpen + pIsClosedNotIsClosed*belClosed;
    }
    else{
      cout << "Wrong action!\n";
      continue;
    }

    // Sensor measurement
    cout << "What did the sensor read? (o / c) : ";
    cin >> sensor;

    // Calculate belOpen and belClosed without the normalizer constant, for now
    if (sensor == "o"){
      belOpen = pSOpenIsOpen*belBarOpen;
      belClosed = pSOpenIsClosed*belBarClosed;
    }
    else if (sensor == "c") {
      belOpen = norm*pSClosedIsOpen*belBarOpen;
      belClosed = norm*pSClosedIsClosed*belBarClosed;
    }
    else {
      continue;
    }

    // Normalize the belOpen and belClosed
//    norm = 1/(belOpen + belClosed);
    norm = pow(belOpen + belClosed, -1);
    belOpen = norm*belOpen;
    belClosed = norm*belClosed;

    // Final results
    cout << "Belieft that the door is now open: " << belOpen << endl;
    cout << "Belieft that the door is now closed: " << belClosed << endl;
    cout << "End of iteration #: " << t << endl;

  }

  return 0;
}
