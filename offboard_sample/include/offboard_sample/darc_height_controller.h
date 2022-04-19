/*
 * @Author: xindong324
 * @Date: 2022-03-06 20:27:40
 * @LastEditors: xindong324
 * @LastEditTime: 2022-03-28 18:57:33
 * @Description: file content
 */
#ifndef _DARC_HEIGHT_CONTROLLER_H
#define _DARC_HEIGHT_CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Eigen>

using namespace std;

struct UavParameters{
	double mass;
	double K_fz; // lift coeff
	double V_s;
};

class DARCHeightController
{
public:
	DARCHeightController(ros::NodeHandle &nh, double dt);
	~DARCHeightController();

	static double DARCAltitudeControl();
	void nl_projection();

private:
	double alpha_z_target_;
	double alpha_z_dot_target_;
	double time_constant_;

	//double tau_z_{0.2};
	//double tau_z_dot_;
	double k_fz_, v_z_, mass_;
	double k_l_z_;
	double k_s1_z_;
	double pos_target_z_, pos_target_z_filtered_, pos_target_z_filtered_old_;
	double ang_ef_target_z_;
	double z_eq_dot_old;
	double vel_target_z_, vel_target_z_filtered_;
	double target_vel_, target_vel_filtered_;
	double z_eq_dot_old_;
	Eigen::Vector3d theta_hat_z_, theta_hat_z_min_, theta_hat_z_max_;
	double h_z_, h_m_z_;
	double k_eq_z_;
	double epsilon_z_;
	double zeta_z_{0.707};
	double k_i_z_;
	Eigen::Vector3d w_z_, phi_z_;
	double s_phi_z_;
	double gamma_z_;
	Eigen::Vector3d Gamma_z_;
	std::vector<double> theta_dot_, theta_hat_, theta_min_, theta_max_;

};

#endif