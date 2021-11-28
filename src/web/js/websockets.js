// Connecting to ROS
  // -----------------
  var ros = new ROSLIB.Ros();

  // If there is an error on the backend, an 'error' emit will be emitted.
  ros.on('error', function(error) {
    console.log(error);
    console.log('Connection made!');
    $("#ros-status").html("error")
    $("#ros-status").css('color', 'red')
  });

  // Find out exactly when we made a connection.
  ros.on('connection', function() {
    console.log('Connection made!');
    $("#ros-status").html("connected")
    $("#ros-status").css('color', 'green')
  });

  ros.on('close', function() {
    console.log('Connection closed.');
    $("#ros-status").html("disconnected")
    $("#ros-status").css('color', 'red')
  });

  // Create a connection to the rosbridge WebSocket server.
  ros.connect('ws://localhost:9090');

    //Subscribing to a Topic
  //----------------------

  // Update complete count
  var countListener = new ROSLIB.Topic({
    ros : ros,
    name : '/complete_count',
    messageType : 'std_msgs/UInt32'
  });

  
  countListener.subscribe(function(message) {
    console.log('Received message on ' + countListener.name + ': ' + message.data);
    $("#complete-count").html(message.data)
  });


  // Update zone status, by changing backgroun
  var zoneStatusListener = new ROSLIB.Topic({
    ros : ros,
    name : '/zone_availability',
    messageType : 'std_msgs/Bool'
  });
  
  zoneStatusListener.subscribe(function(message) {
    console.log('Received message on ' + countListener.name + ': ' + message.data);
    if(message.data)
    {
        $("#zone-status").toggleClass('bg-danger bg-success');
    }
    else
    {
        $("#zone-status").toggleClass('bg-success bg-danger');
    }
  });

  // Update agv1 status
  var agv1StatusListener = new ROSLIB.Topic({
    ros : ros,
    name : '/agv_status',
    messageType : 'std_msgs/String'
  });
  
  agv1StatusListener.subscribe(function(message) {
    console.log('Received message on ' + countListener.name + ': ' + message.data);
    $("#agv1-status").html(message.data)
  });
