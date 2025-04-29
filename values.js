let ros, cmdVelTopic, linearspeed = 0, angularspeed = 0, currentTopicName;
let velocityInterval = null;
let robotPose = { x: 0, y: 0, orientation: 0 }; // Made global for access in LaserScan drawing

function setTopicName() {
    const topicInput = document.getElementById('topicName').value.trim();
    if (topicInput !== "") {
        currentTopicName = topicInput;

        if (ros && ros.isConnected) {
            cmdVelTopic = new ROSLIB.Topic({
                ros: ros,
                name: currentTopicName,
                messageType: 'geometry_msgs/msg/Twist'
            });
        }
    } else {
        alert("Please enter a valid topic name.");
    }
}

function connectToRos() {
    const ipAddress = document.getElementById('ipAddress').value.trim();
    if (!ipAddress) return alert("Enter a valid IP");

    ros = new ROSLIB.Ros({ url: "ws://" + ipAddress + ":9090" });

    ros.on('connection', () => {
        document.getElementById('connectButton').innerText = 'Disconnect';
        document.getElementById('connectButton').classList.add('connected');
        document.getElementById('connectionIndicator').classList.add('connected');

        cmdVelTopic = new ROSLIB.Topic({
            ros: ros,
            name: currentTopicName || "/cmd_vel",
            messageType: 'geometry_msgs/msg/Twist'
        });

        initScanTab();
    });

    ros.on('error', (error) => {
        console.error('Connection error:', error);
        alert("Connection failed. Please check:\n- IP Address\n- ROSBridge running\n- Network\n- Port 9090 open");
    });

    ros.on('close', () => {
        document.getElementById('connectButton').innerText = 'Connect';
        document.getElementById('connectButton').classList.remove('connected');
        document.getElementById('connectionIndicator').classList.remove('connected');
    });
}

function toggleConnection() {
    if (ros && ros.isConnected) ros.close();
    else connectToRos();
}

document.getElementById('linearspeedRange').addEventListener('input', (e) => {
    linearspeed = parseFloat(e.target.value);
    document.getElementById('linearspeedValue').innerText = linearspeed;
});

document.getElementById('angularspeedRange').addEventListener('input', (e) => {
    angularspeed = parseFloat(e.target.value);
    document.getElementById('angularspeedValue').innerText = angularspeed;
});

function callSetBoolService(value) {
    if (!ros || !ros.isConnected) {
        alert("ROS is not connected. Please connect first.");
        return;
    }

    const path = document.getElementById('messageInput').value.trim();
    if (value && path === "") {
        alert("Please enter a path before starting.");
        return;
    }

    const boolService = new ROSLIB.Service({
        ros: ros,
        name: '/startStopRecording',
        serviceType: '/si_slam_interfaces/srv/StartStopRecording'
    });

    const request = value ? { start: true, path: path } : { start: false };
    const serviceRequest = new ROSLIB.ServiceRequest(request);

    boolService.callService(serviceRequest, (result) => {
        alert("Service called. Success: " + result.success + " | Message: " + result.message);
    });
}

document.getElementById('enableButton').addEventListener('click', () => callSetBoolService(true));
document.getElementById('disableButton').addEventListener('click', () => callSetBoolService(false));

const activeKeys = new Set();

document.addEventListener('keydown', (e) => {
    if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(e.key)) {
        e.preventDefault();
        const initialSize = activeKeys.size;
        activeKeys.add(e.key);

        if (initialSize === 0) {
            startPublishing();
        }
    }
});

document.addEventListener('keyup', (e) => {
    if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(e.key)) {
        activeKeys.delete(e.key);

        if (activeKeys.size === 0) {
            stopPublishing();
        }
    }
});

function startPublishing() {
    if (!velocityInterval) {
        publishVelocity();  
        velocityInterval = setInterval(publishVelocity, 100); 
    }
}

function stopPublishing() {
    if (velocityInterval) {
        clearInterval(velocityInterval);
        velocityInterval = null;
        publishZeroVelocity(); 
    }
}

function publishVelocity() {
    if (!ros || !ros.isConnected || !cmdVelTopic) return;

    let linearX = 0;
    let angularZ = 0;

    const forward = activeKeys.has("ArrowUp");
    const reverse = activeKeys.has("ArrowDown");
    const turnLeft = activeKeys.has("ArrowLeft");
    const turnRight = activeKeys.has("ArrowRight");

    if (forward) linearX = linearspeed;
    else if (reverse) linearX = -linearspeed;

    if (turnLeft) {
        angularZ = reverse ? -angularspeed : angularspeed;
    }
    if (turnRight) {
        angularZ = reverse ? angularspeed : -angularspeed;
    }

    const moveMsg = new ROSLIB.Message({
        linear: { x: linearX, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularZ }
    });

    cmdVelTopic.publish(moveMsg);
}

function publishZeroVelocity() {
    if (!ros || !ros.isConnected || !cmdVelTopic) return;

    const stopMsg = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
    });

    cmdVelTopic.publish(stopMsg);
}

function initScanTab() {
    if (!ros || !ros.isConnected) return;

    const poseListener = new ROSLIB.Topic({
        ros: ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
    });

    poseListener.subscribe((msg) => {
        const p = msg.pose.pose.position;
        const o = msg.pose.pose.orientation;
        robotPose = { x: p.x, y: p.y, orientation: quaternionToYaw(o) };

        const poseText = `Pose → X: ${robotPose.x.toFixed(2)} | Y: ${robotPose.y.toFixed(2)} | θ: ${robotPose.orientation.toFixed(2)}`;
        const poseElem = document.getElementById('pose');
        if (poseElem) poseElem.innerText = poseText;
    });

    const laserListener = new ROSLIB.Topic({
        ros: ros,
        name: '/scan',
        messageType: 'sensor_msgs/LaserScan'
    });

    laserListener.subscribe((msg) => {
        const canvas = document.getElementById('laserCanvas');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        const scale = 70;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;

        msg.ranges.forEach((range, i) => {
            if (range > msg.range_min && range < msg.range_max) {
                const angle = msg.angle_min + i * msg.angle_increment;
                const x = range * Math.cos(angle);
                const y = range * Math.sin(angle);

                const rotX = x * Math.cos(robotPose.orientation) - y * Math.sin(robotPose.orientation);
                const rotY = x * Math.sin(robotPose.orientation) + y * Math.cos(robotPose.orientation);

                ctx.beginPath();
                ctx.arc(centerX + rotX * scale, centerY - rotY * scale, 1.5, 0, 2 * Math.PI);
                ctx.fillStyle = 'red';
                ctx.fill();
            }
        });
    });
}

function quaternionToYaw(q) {
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny_cosp, cosy_cosp);
}