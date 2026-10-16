var listener = new ROSLIB.Topic({
  ros: ros,
  name: '/Converter_node',
  messageType: 'std_msgs/msg/String'
});

listener.subscribe(function(message) {
  console.log('Received message on ' + listener.name + ': ' + message.data);
  listener.unsubscribe();
});


