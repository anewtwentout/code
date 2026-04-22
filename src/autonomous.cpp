#include "main.h"
#include "auto.hpp"
#include "lemlib/api.hpp"
enum autonomousEnums {SAWPAUTO = 0, rightControlRushAUTO = 1, leftControlRushAUTO = 2, rightFourPlusThreeAUTO = 3, leftThreePlusFourAUTO = 4, rightSevenAUTO = 5, leftSevenAUTO = 6};
int autonomousValue = rightFourPlusThreeAUTO;
void autonomous() {
   switch(autonomousValue){
    case SAWPAUTO:
     SAWP();
     break;

    case rightControlRushAUTO:
    rightControlRush();
    break;

    case leftControlRushAUTO:
    leftControlRush();
    break;

    case rightFourPlusThreeAUTO:
    rightFourPlusThree();
    break;

    case leftThreePlusFourAUTO:
    leftThreePlusFour();
    break;

    case rightSevenAUTO:
    rightSeven();
    break;
    
    case leftSevenAUTO:
    leftSeven();
    break;
   }
     
 }