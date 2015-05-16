#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <ros/xmlrpc_manager.h>

#include <signal.h>

#include <arm_bootloader/arm_bootloader.h>

#include <stm32f3discovery_imu_driver/protocol.h>

extern unsigned char firmware_bin[];
extern int firmware_bin_len;

using namespace stm32f3discovery_imu_driver;
using namespace std;

// Functionality to allow a member function to be used in place of a void (*func_ptr)(int)
// 		See http://stackoverflow.com/questions/13238050/convert-stdbind-to-function-pointer
// -------------------------------------------------------------------

template<unsigned ID,typename Functor>
boost::optional<Functor> &get_local()
{
    static boost::optional<Functor> local;
    return local;
}

template<unsigned ID,typename Functor>
typename Functor::result_type wrapper(int sig)
{
    return get_local<ID,Functor>().get()(sig);
}

template<typename ReturnType>
struct Func
{
    typedef ReturnType (*type)(int);
};

template<unsigned ID,typename Functor>
typename Func<typename Functor::result_type>::type get_wrapper(Functor f)
{
    (get_local<ID,Functor>()) = f;
    return wrapper<ID,Functor>;
}

// ----------------------------------------------------------------------


class interface
{
private:	// Vars
	// Random num gen
	std::mt19937 gen_;
	std::uniform_int_distribution<> dis_;

	// Bootloader
	arm_bootloader::Dest dest_;
	boost::shared_ptr<arm_bootloader::Reader<Response>> reader_;

	// Protocol
	boost::shared_ptr<uf_subbus_protocol::ChecksumAdder<uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink> >>
			checksumadder_;

	// Pwms
	double pwm1_, pwm2_;
	double zero_pwm_;

	// Publishers and subscribers
	ros::Publisher imu_pub_, mag_pub_;
	ros::Subscriber pwm1_sub_, pwm2_sub_;

	// TF
	std::string frame_id_;

private:	// Functions
	void writePwms_();
	void setPwm_(double &pwm, std_msgs::Float64ConstPtr msg);
	void onShutdown_();
	void xmlrpcShutdown_(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
	void sigintShutdown_(int sig);

public: 	// Functions
	interface(int argc, char **argv);
	void run_();

};

interface::interface(int argc, char **argv)
{
	// Initialize ROS with an asynchronism spinner
	ros::init(argc, argv, "stm32f3discovery_imu_driver", ros::init_options::NoSigintHandler);
	ros::NodeHandle private_nh("~");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

	// Get the zero pwm
	private_nh.param<double>("zero_pwm", zero_pwm_, 1.5e-3);
	pwm1_ = zero_pwm_;
	pwm2_ = zero_pwm_;

	// Setup kill signals
		// Convert from member function to void (*)(int)
	boost::function<void (int)> boost_f = boost::bind(&interface::sigintShutdown_, this, _1);
	void (*c_f)(int) = get_wrapper<1>(boost_f);
	signal(SIGINT, c_f);		// ctr-c (or any other SIGINT producer)
	ros::XMLRPCManager::instance()->unbind("shutdown");				// rosnode kill cmd line tool
	boost::function<void(XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&)> f = boost::bind(&interface::xmlrpcShutdown_, this, _1, _2);
	ros::XMLRPCManager::instance()->bind("shutdown", f);

	// Setup IO
	boost::asio::io_service io;
	boost::asio::serial_port sp(io);
	std::string port; private_nh.getParam("port", port);
	sp.open(port);
	sp.set_option(boost::asio::serial_port::baud_rate(115200));

	std::string deststr; private_nh.getParam("dest", deststr);
	dest_ = strtol(deststr.c_str(), NULL, 0);

	// Get the frame id
	private_nh.getParam("frame_id", frame_id_);

	// Run the bootloader
	if(argc <= 1) {
		if(!arm_bootloader::attempt_bootload(port, sp, dest_, firmware_bin, firmware_bin_len)) {
			ROS_ERROR("bootloading failed");
			ros::shutdown();
			return;
		}
	}

	// Generate random number
	std::random_device rd;
	gen_ = std::mt19937(rd());
	dis_ = std::uniform_int_distribution<>(1, 65535);

	// Make a reader
	reader_ = boost::shared_ptr<arm_bootloader::Reader<Response> >
		(new arm_bootloader::Reader<Response>(sp, 1000));

	// Setup protocol
	arm_bootloader::SerialPortSink sps(port, sp);
	uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink>
		packetizer(sps);
	checksumadder_ =
			boost::shared_ptr<uf_subbus_protocol::ChecksumAdder<uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink> >>
			(new uf_subbus_protocol::ChecksumAdder<uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink> >(packetizer));

	// Set up publishers and subscribers
	imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10);
	mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("/imu/mag_raw", 10);

	pwm1_sub_ = private_nh.subscribe<std_msgs::Float64>("pwm1", 10,
			boost::bind(&interface::setPwm_, this, pwm1_, _1) );
	pwm2_sub_ = private_nh.subscribe<std_msgs::Float64>("pwm2", 10,
			boost::bind(&interface::setPwm_, this, pwm2_, _1) );
}

void interface::setPwm_(double &pwm, std_msgs::Float64ConstPtr msg)
{
	pwm = msg->data;
}

void interface::writePwms_()
{
	Command cmd; memset(&cmd, 0, sizeof(cmd));
	cmd.dest = dest_;
	cmd.id = dis_(gen_);
	cmd.command = CommandID::SetPWM;
	cmd.args.SetPWM.length[0] = pwm1_;
	cmd.args.SetPWM.length[1] = pwm2_;
	write_object(cmd, (*checksumadder_));

	boost::optional<Response> resp = reader_->read(cmd.id);
	if(!resp) {
		ROS_WARN("Timeout receiving pwm packet");
	}
}

void interface::onShutdown_()
{
	// Stop both subscribers in case somehow a msg comes between setting pwms to zero and the write pwm cmd
	pwm1_sub_.shutdown();
	pwm2_sub_.shutdown();

	// Zero pwms
	pwm1_ = zero_pwm_;
	pwm2_ = zero_pwm_;

	// Write the zeros
	writePwms_();

	// Shutdown ros
	ros::shutdown();
}

void interface::sigintShutdown_(int sig)
{
	onShutdown_();
}

// This is only called when using rosnode kill
void interface::xmlrpcShutdown_(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int num_params = 0;
	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		num_params = params.size();
	}

	if (num_params > 1)
	{
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		onShutdown_();
	}

	result = ros::xmlrpc::responseInt(1, "", 0);
}

void interface::run_()
{
	while(ros::ok()) {
		Command cmd; memset(&cmd, 0, sizeof(cmd));
		cmd.dest = dest_;
		cmd.id = dis_(gen_);
		cmd.command = CommandID::GetIMUData;
		write_object(cmd, (*checksumadder_));

		boost::optional<Response> resp = reader_->read(cmd.id);
		if(!resp) {
			ROS_WARN("Timeout receiving imu packet!");
			continue;
		}

		ros::Time now = ros::Time::now();

		sensor_msgs::Imu msg;
		msg.header.stamp = now;
		msg.header.frame_id = frame_id_;
		msg.linear_acceleration.x = resp->resp.GetIMUData.linear_acceleration[0];
		msg.linear_acceleration.y = resp->resp.GetIMUData.linear_acceleration[1];
		msg.linear_acceleration.z = resp->resp.GetIMUData.linear_acceleration[2];
		msg.angular_velocity.x = resp->resp.GetIMUData.angular_velocity[0];
		msg.angular_velocity.y = resp->resp.GetIMUData.angular_velocity[1];
		msg.angular_velocity.z = resp->resp.GetIMUData.angular_velocity[2];
		imu_pub_.publish(msg);

		sensor_msgs::MagneticField msg2;
		msg2.header.stamp = now;
		msg2.header.frame_id = frame_id_;
		msg2.magnetic_field.x = resp->resp.GetIMUData.magnetic_field[0];
		msg2.magnetic_field.y = resp->resp.GetIMUData.magnetic_field[1];
		msg2.magnetic_field.z = resp->resp.GetIMUData.magnetic_field[2];
		mag_pub_.publish(msg2);

		writePwms_();
	}
}

int main(int argc, char **argv)
{
	interface node(argc, argv);

	node.run_();

	ros::waitForShutdown();
	return EXIT_SUCCESS;
}
