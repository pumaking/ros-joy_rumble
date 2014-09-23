/*
 * rumble.cpp
 *
 *  Created on: 22.09.2014
 *      Author: fnolden
 */

#include "rumble.h"
#define max(a,b) ((a)>(b)?(a):(b))

Rumble::Rumble() {
	// TODO Auto-generated constructor stub
	update_rate = 5;
	ff_fd = -1;
	ptr_imu_msg_old = 0;
	ptr_effect_buffer = 0;
	nr_simultaneous_effects = 0;
	node_name = "joy_rumble";
	//imu_old = NULL;
}

void
Rumble::init(ros::NodeHandle &nh){
	nh.param<std::string>("event", event, "/dev/input/event6");
	nh.param<int>("intervall", update_rate, 5);
	nh.param<std::string>("sub_topic_imu", sub_topic_imu, "/imu_message");
	nh.param<std::string>("sub_topic_rumble", sub_topic_rumble, "/rumble_message");

	sub_imu = nh.subscribe(sub_topic_imu.c_str(), 3, &Rumble::callback_imu, this);
	sub_rumble = nh.subscribe(sub_topic_rumble.c_str(), 3, &Rumble::callback_rumble, this);
	open_device();

}

bool Rumble::open_device(){
	if(ff_fd != -1) close(ff_fd); // if a device is already open, close it first
	ff_fd = open(event.c_str(), O_RDWR);

	if (ff_fd == -1) {
		ROS_ERROR_NAMED("joy_rumble","Open device \"%s\" failed", event.c_str());
		return false;
	}
	if (ioctl(ff_fd, EVIOCGEFFECTS, &nr_simultaneous_effects) < 0) {
		ROS_ERROR_NAMED(node_name,"Ioctl number of effects");
		return false;
	}
	ptr_effect_buffer = new boost::circular_buffer<input_event>(nr_simultaneous_effects);

	ROS_INFO_NAMED(node_name,"Open device \"%s\" succeeded, %i simultaneous effects supported", event.c_str(), nr_simultaneous_effects);
	write_force(0x00f0, 0x0, 0x0, 0x23ff);
	return true;
}

void
Rumble::callback_imu(const sensor_msgs::Imu::ConstPtr &msg ){
	//Do something here
	//

	//ptr->angular_velocity.x

	//imu_old = ptr;
	ptr_imu_msg_old = 0;
	ptr_imu_msg_old = new boost::shared_ptr<const sensor_msgs::Imu>(msg);
}

void
Rumble::callback_rumble(const joy_rumble::Rumble_msg::ConstPtr &msg ){
	write_force(msg->length, msg->direction, msg->weak, msg->strong);
}

void
Rumble::write_force(uint16_t length, uint16_t direction, uint16_t weak, uint16_t strong ){
	ROS_INFO_NAMED(node_name, "length: %04x direction: %04x weak: %04x strong: %04x",
			length, direction,
			weak, strong
			);

	ff_effect effect;

	effect.type = FF_RUMBLE;
	effect.direction = direction;

	effect.u.rumble.strong_magnitude = strong;
	effect.u.rumble.weak_magnitude = weak;

	effect.replay.length = length;
	effect.replay.delay = 0;
	effect.id = -1;

	if (ioctl(ff_fd, EVIOCSFF, &effect) < 0) {
	/* If updates are sent to frequently, they can be refused */
		ROS_INFO_NAMED(node_name, "to many updates?");
		//
	}

	struct input_event play, stop;
	play.type = EV_FF;
	play.code = effect.id;
	play.value = 1;
	if (write(ff_fd, (const void*) &play, sizeof(play)) == -1) {
		ROS_ERROR_NAMED(node_name,"Play effect");
		exit(1);
	}

	stop.type = EV_FF;
	stop.code = effect.id;
	stop.value = 0; //start = 1, stop = 0


	ptr_effect_buffer->push_back(stop);
	if(ptr_effect_buffer->full()){
		//delete last effect from controller memory
		//ROS_INFO_NAMED(node_name, "stopping %i", ptr_effect_buffer->begin()->code);
//		if (write(ff_fd, (const void*) &ptr_effect_buffer->front(), sizeof(ptr_effect_buffer->front())) == -1) {
//			ROS_ERROR_NAMED(node_name,"Stop effect");
//			exit(1);
//		}
		if (ioctl(ff_fd, EVIOCRMFF, ptr_effect_buffer->front().code) == -1){
			ROS_ERROR_NAMED(node_name,"Stop effect");
			exit(1);
		}

		ptr_effect_buffer->pop_front();
	}
	//ROS_INFO_NAMED(node_name, "playing %i front: %i", effect.id, ptr_effect_buffer->front().code);
}


Rumble::~Rumble() {
	// TODO Auto-generated destructor stub
	if(ff_fd != -1) close(ff_fd);
}

