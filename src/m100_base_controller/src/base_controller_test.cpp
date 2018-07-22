#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controller_test");
    ros::NodeHandle n;
   

    DJIDrone* drone = new DJIDrone(n);
    drone->request_sdk_permission_control();
    sleep(1);
    drone->takeoff();
    sleep(1);

    ros::Rate r(10);
    int i = 0;

    while(n.ok()) {
        ros::spinOnce();
        i ^= 1;
        if (i == 0) {
            drone->attitude_control(0x5b, 0, 1, 1.5, 0);
	    r.sleep();
        }
        else {
            drone->attitude_control(0x5b, 0, 0, 1.5, 0);
	    r.sleep();
        }
    }
    return 0;
}
