// complete_mission.cpp
#include <complete_mission.hpp>

int mission_num = 0;
ros::Time  hover_request_time;

int main(int argc, char **argv)
{
    // 防止中文输出乱码
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "complete_mission");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    // 发布无人机多维控制话题
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

    // 创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // 创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(20);

    // 等待连接到飞控
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // 设置无人机的期望位置
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    // 定义客户端变量，设置为offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 定义客户端变量，请求无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 记录当前时间，并赋值给变量last_request
    ros::Time last_request = ros::Time::now();
    bool hover_started =false;
    ros::Time reach_time= ros::Time::now();
    int last_mission_num=-1;

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.1)
        {
            if(!hover_started)
            {
		
		hover_request_time = ros::Time::now();
		hover_started =true;
		ROS_INFO("starting hover timer for 12 seconds");
	    }
	    	
            if (ros::Time::now() - hover_request_time > ros::Duration(12.0))
            {
                mission_num = 1;
                break;
            }
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, 0.3);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }


  while (ros::ok())
  {
    // 每次循环检查是否需要重置计时
    if(last_mission_num != mission_num)
    {
        reach_time = ros::Time::now();
        last_mission_num = mission_num;
    }
    
    switch (mission_num)
    {
    case 1:
        // 飞到点A (1, 0)
        if (mission_pos_cruise(1.0, -1.5, ALTITUDE, 0, 0.4))
        {
            // 稳定至少2秒再切换下一个航点
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 2;
            }
        }
   
        break;

    case 2:
        // 飞到点B (1, -1)
        if (mission_pos_cruise(4.0, -1.5, ALTITUDE, 0, 0.4))
        {
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 3;
            }
        }
     
        break;

    case 3:
        // 飞到点C (10, -1)
        if (mission_pos_cruise(5.0, -1.5, ALTITUDE, 0, 0.4))
        {
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 4;
            }
        }
     
        break;

    case 4:
        // 点 (11, -0.4)
        if (mission_pos_cruise(8.0, -1.0, ALTITUDE, 0, 0.3))
        {
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 5;
            }
        }
     
        break;
        
    case 5:
    	if (mission_pos_cruise(9.8, -0.9, ALTITUDE, 0, 0.2))
        {
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 6;
            }
        }
     
        break;
      
    case 6:
    	if (mission_pos_cruise(10.2, -1.0, ALTITUDE, 0, 0.15))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 7;
            }
        }
     
    	break;
    	
    case 7:
    	if (mission_pos_cruise(11.0, -0.4, ALTITUDE, 0,0.3))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 8;
            }
        }
      
    	break;
    	
    case 8:
    	if (mission_pos_cruise(14.0, 0.0,  ALTITUDE,0 ,0.4))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 9;
            }
        }
     
    	break;	
    	
    case 9:
      	if (mission_pos_cruise(15.8, 1.0,  ALTITUDE,0, 0.4))
      	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 10;
            }
        }
    
      	break;
      	    
    case 10:
    	if (mission_pos_cruise(17.0, 2.4, ALTITUDE, 0, 0.2))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 11;
            }
        }
      
    	break;
    
    case 11:
    	if (mission_pos_cruise(18.0, 2.4, ALTITUDE, 0, 0.2))
    	{
    		if(ros::Time::now()-reach_time>ros::Duration(2.0))
    		{
    			mission_num=12;
    		}
    	}
    	break;		
    	   
    case 12:
    	if (mission_pos_cruise(19.6, 2.4, ALTITUDE, 0, 0.15))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 13;
            }
        }
     
        break;
    	   
    case 13:
    	if (mission_pos_cruise(20.0, 0.2, ALTITUDE, 0, 0.2))
        {
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 14;
            }
        }
    
        break;
        
    case 14:
    	if( mission_pos_cruise(21.0, 0.2, ALTITUDE, 0, 0.15))
    	{
    		if(ros::Time::now()-reach_time>ros::Duration(2.0))
    		{
			mission_num=15;
		}
	}
	break;
        
    case 15:
    	if (mission_pos_cruise(23.0, 0.2, ALTITUDE, 0, 0.15))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 16;
            }
        }
     
    	break;       
    	
    case 16:
    	if (mission_pos_cruise(25.0, 1.2, ALTITUDE, 0, 0.3))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 17;
            }
        }
      
    	break;
    	  
    case 17:
    	if (mission_pos_cruise(29.0, 1.2, ALTITUDE, 0, 0.3))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 18;
            }
        }
       
    	break;
    	
    case 18:
    	if (mission_pos_cruise(34.6, 1.0, ALTITUDE, 0, 0.2))
    	{
            if (ros::Time::now() - reach_time > ros::Duration(2.0))
            {
                mission_num = 19;
            }
        }
        
    	break;

    case 19:
        ROS_INFO("AUTO.LAND");
        offb_set_mode.request.custom_mode = "AUTO.LAND";
        set_mode_client.call(offb_set_mode);
        mission_num = -1;
        break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
}
