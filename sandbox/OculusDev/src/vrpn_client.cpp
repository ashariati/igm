#include "../../vrpn/vrpn_Tracker.h"
#include "../../vrpn/vrpn_Button.h"
#include "../../vrpn/vrpn_Analog.h"

#include <iostream>
using namespace std;



void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t )
{
cout << "Tracker '" << t.sensor << "' : " << t.pos[0] << "," <<  t.pos[1] << "," << t.pos[2] << endl;
}


int main(int argc, char* argv[])
{

    // vrpn_Tracker_Remote* vrpnTracker = new vrpn_Tracker_Remote( "Wand@158.130.62.132:3883");
	vrpn_Tracker_Remote* vrpnTracker = new vrpn_Tracker_Remote( "Wand@158.130.62.126:3883");
	vrpnTracker->register_change_handler( 0, handle_tracker );

while(1)
{

        vrpnTracker->mainloop();
}

return 0;
}
