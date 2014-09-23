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
	ptr_effect_buffer = 0;
	nr_simultaneous_effects = 0;
	node_name = "joy_rumble";
	max_delta_acc = 0;
	max_delta_velo = 0;
	//imu_old = NULL;
}

void
Rumble::init(ros::NodeHandle &nh){
	nh.param<std::string>("event", event, "/dev/input/event6");
	nh.param<int>("intervall", update_rate, 5);
	nh.param<std::string>("sub_topic_imu", sub_topic_imu, "/imu/data");
	nh.param<std::string>("sub_topic_rumble", sub_topic_rumble, "/rumble_message");
	nh.param<double>("velo_delta_min", velo_delta[0], 0.05);
	nh.param<double>("velo_delta_max", velo_delta[1], 0.15);
	nh.param<double>("acc_delta_min", acc_delta[0], 0.5);
	nh.param<double>("acc_delta_max", acc_delta[1], 2.0);
	int temp_rumble =0;
	nh.param<int>("rumble_min", temp_rumble,  32768);
	min_rumble = (uint16_t) temp_rumble;

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
	write_force(0x0120, 0x0, 0x0, 0x23ff);
	return true;
}

uint16_t
Rumble::calculate_force(double delta, double min, double max){
	uint16_t min_rumble = 0x8000;
	if(delta < min) return 0x0;
	else if(delta >= max ) return 0xffff;
	else{
		uint16_t scale = 0xffff - min_rumble;
		uint16_t result = min_rumble + (scale / (max - min))*delta;
		//ROS_INFO_NAMED("joy_rumble","Delta: %f min: %f max %f %i", delta, min, max, result);
		return result;
	}
}

void
Rumble::callback_imu(const sensor_msgs::Imu::ConstPtr &msg ){
	//Do something here
	//

	double delta[6];

	delta[0] = fabs(msg->angular_velocity.x - imu_velo_old[0]);
	delta[1] = fabs(msg->angular_velocity.y - imu_velo_old[1]);
	delta[2] = fabs(msg->angular_velocity.z - imu_velo_old[2]);
	//ROS_INFO_NAMED("joy_rumble","Velx: %f old Velx: %f delta %f", msg->angular_velocity.x, imu_velo_old[0], delta[0]);

	delta[3] = fabs(msg->linear_acceleration.x - imu_acc_old[0]);
	delta[4] = fabs(msg->linear_acceleration.y - imu_acc_old[1]);
	delta[5] = fabs(msg->linear_acceleration.z - imu_acc_old[2]);

	uint16_t force[6];
	uint16_t final = 0x0;
	force[0] = calculate_force(delta[0],velo_delta[0],velo_delta[1]);
	force[1] = calculate_force(delta[1],velo_delta[0],velo_delta[1]);
	force[2] = calculate_force(delta[2],velo_delta[0],velo_delta[1]);

	force[3] = calculate_force(delta[3],acc_delta[0],acc_delta[1]);
	force[4] = calculate_force(delta[4],acc_delta[0],acc_delta[1]);
	force[5] = calculate_force(delta[5],acc_delta[0],acc_delta[1]);

	for(int i=0; i<3 ; i++ ){
		final = max(final, force[i]);
		final = max(final, force[i+3]);
		max_delta_velo = fmax(max_delta_velo, delta[i]);
		max_delta_acc = fmax(max_delta_acc, delta[i+3]);
	}

	//ROS_INFO_NAMED("joy_rumble","max %i, delta max, velo: %f acc: %f", final, max_delta_velo, max_delta_acc );

	if(final > min_rumble )	write_force(0x0100,0x0,0x0,final);

	imu_velo_old[0] = msg->angular_velocity.x;
	imu_velo_old[1] = msg->angular_velocity.y;
	imu_velo_old[2] = msg->angular_velocity.z;

	imu_acc_old[0] = msg->linear_acceleration.x;
	imu_acc_old[1] = msg->linear_acceleration.y;
	imu_acc_old[2] = msg->linear_acceleration.z;
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

