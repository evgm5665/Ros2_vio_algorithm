var cmdVel = new ROSLIB.Topic({
  ros: ros,
  name: '/output_msg',
  messageType: 'px4_msgs/msg/VehicleOdometry'
});

var twist = new ROSLIB.Message({
  linear: { x: 0.5, y: 0.0, z: 0.0 },
  angular: { x: 0.0, y: 0.0, z: 0.2 }
});

cmdVel.publish(twist);
