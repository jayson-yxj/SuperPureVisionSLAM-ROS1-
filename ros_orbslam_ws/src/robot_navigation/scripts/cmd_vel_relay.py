#!/usr/bin/env python3
"""
速度命令转发节点
功能：订阅 move_base 发布的 /cmd_vel，转发给机器人底盘
"""

import rospy
from geometry_msgs.msg import Twist

class CmdVelRelay:
    def __init__(self):
        rospy.init_node('cmd_vel_relay', anonymous=False)
        
        # 参数配置
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.5)   # 最大线速度（米/秒）
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0) # 最大角速度（弧度/秒）
        self.enable_safety_check = rospy.get_param('~enable_safety_check', True)
        
        # 订阅 move_base 的速度命令
        self.cmd_vel_sub = rospy.Subscriber(
            '/cmd_vel', 
            Twist, 
            self.cmd_vel_callback, 
            queue_size=10
        )
        
        # 发布到机器人底盘（根据实际机器人修改话题名）
        # 示例：如果使用 TurtleBot3，话题可能是 /cmd_vel
        # 如果使用其他机器人，修改为对应的话题
        self.robot_cmd_pub = rospy.Publisher(
            '/robot/cmd_vel',  # 修改为实际机器人的速度命令话题
            Twist, 
            queue_size=10
        )
        
        # 安全停止定时器（如果长时间没有收到命令，自动停止）
        self.last_cmd_time = rospy.Time.now()
        self.safety_timeout = rospy.Duration(1.0)  # 1秒超时
        self.safety_timer = rospy.Timer(rospy.Duration(0.1), self.safety_check)
        
        rospy.loginfo("✓ 速度命令转发节点已启动")
        rospy.loginfo(f"  订阅: /cmd_vel")
        rospy.loginfo(f"  发布: /robot/cmd_vel")
        rospy.loginfo(f"  速度限制: 线速度={self.max_linear_vel}m/s, 角速度={self.max_angular_vel}rad/s")
    
    def cmd_vel_callback(self, msg):
        """
        接收速度命令并转发
        """
        # 更新最后接收时间
        self.last_cmd_time = rospy.Time.now()
        
        # 速度限制（安全检查）
        cmd = Twist()
        cmd.linear.x = self.clamp(msg.linear.x, -self.max_linear_vel, self.max_linear_vel)
        cmd.linear.y = self.clamp(msg.linear.y, -self.max_linear_vel, self.max_linear_vel)
        cmd.linear.z = 0.0  # 地面机器人不使用 Z 轴
        
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.clamp(msg.angular.z, -self.max_angular_vel, self.max_angular_vel)
        
        # 发布到机器人
        self.robot_cmd_pub.publish(cmd)
        
        # 日志（限流）
        rospy.loginfo_throttle(
            2, 
            f"📡 速度命令: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}"
        )
    
    def safety_check(self, event):
        """
        安全检查：如果长时间没有收到命令，发送停止命令
        """
        if not self.enable_safety_check:
            return
        
        time_since_last_cmd = rospy.Time.now() - self.last_cmd_time
        
        if time_since_last_cmd > self.safety_timeout:
            # 发送停止命令
            stop_cmd = Twist()
            self.robot_cmd_pub.publish(stop_cmd)
            rospy.logwarn_throttle(5, "⚠️  长时间未收到速度命令，已发送停止指令")
    
    @staticmethod
    def clamp(value, min_val, max_val):
        """
        限制数值范围
        """
        return max(min_val, min(value, max_val))

if __name__ == '__main__':
    try:
        relay = CmdVelRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("速度命令转发节点已关闭")
