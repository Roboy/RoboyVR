Developer's Manual
==================

Are you sure you want to go down that road? This will be though, are you prepared? Yeah?
So follow me if you want to bring the BeRoboy™ development forth.

*Note: We assume that you already have gone through the User's Manual to not repeat ourselves.*


Gazebo Simulation
-----------------
For the gazebo part you need to create a launch .world file. When this file is launched it automa-
tically loads the world (with all the surrounding objects) and the version of roboy you have chosen.

**Example for a world (camera.world):**

<world name="default">

        <!-- A ground plane -->
        <include>
            <uri>model://ground_plane</uri>
        </include>
	<!--PabiRoboy -->
	<include>
	    <uri>model://legs_with_upper_body</uri>
	</include>
	<!--Sun -->
        <include>
	    <uri>model://sun</uri>
	</include>


        <!-- Focus camera on tall pendulum -->
        <gui fullscreen='0'>
            <camera name='user_camera'>
                <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
                <view_controller>orbit</view_controller>
            </camera>
        </gui>
</world>

**Example for the launch file:**

<launch>
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
	<arg name="world_name" value="$(find roboy_simulation)/worlds/camera.world"/>
	<arg name="paused" value="false"/>
	<arg name="use_sim_time" value="true"/>
	<arg name="gui" value="true"/>
	<arg name="headless" value="false"/>
    	<arg name="debug" value="false"/>
    </include>
</launch>

Model Configuration
-------------------
If you want to see a camera feed from a gazebo simulation you need to have a *camera sensor* that
captures images and publishes them via messages over a ros bridge. Those messages a standard
sensor messages. You can refer to a *gazebo plugin* that has already been implemented. It is re-
commended to attach this sensor to a position close to the models head because you want to its
POV to maximize the POV experience. To implement such a thing, just open the model.sdf of the
specific model you want to have in the simulation and add the following section.


<sensor type="camera" name="camera">
	      <update_rate>1.0</update_rate>
	      <camera name="head">
		<pose>0 1.25 0 -1.5707963267948966 -1.5707963267948966 0</pose>
		<horizontal_fov>1.6962634</horizontal_fov>
		<image>
		  <width>640</width>
		  <height>480</height>
		  <format>R8G8B8</format>
		</image>
		<clip>
		  <near>0.1</near>
		  <far>100</far>
		</clip>
		<noise>
		  <type>gaussian</type>
		  <!-- Noise is sampled independently per pixel on each frame.
		       That pixel's noise value is added to each of its color
		       channels, which at that point lie in the range [0,1]. -->
		  <mean>0.0</mean>
		  <stddev>0.007</stddev>
		</noise>
	      </camera>
	      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
		<alwaysOn>true</alwaysOn>
		<updateRate>0.0</updateRate>
		<cameraName>roboy/camera</cameraName>
		<imageTopicName>image_raw</imageTopicName>
		<cameraInfoTopicName>camera_info</cameraInfoTopicName>
		<frameName>camera_link</frameName>
		<hackBaseline>0.07</hackBaseline>
		<distortionK1>0.0</distortionK1>
		<distortionK2>0.0</distortionK2>
		<distortionK3>0.0</distortionK3>
		<distortionT1>0.0</distortionT1>
		<distortionT2>0.0</distortionT2>
	      </plugin>
</sensor>

The pose determines where the camera will be looking at and which perspective it will be publishing messages from.
In order to publish images the camera sensor needs a plugin attached to it, in this case its a standard plugin-in,
the ros camera from the gazebo library. The width and height tag determine the resolution of the published images,
the update rates is crucial to how many images are sent in one second (25 means, 25 updates per second).

Unity Scene
-----------
bli