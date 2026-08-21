ROBOT = "go2" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml" # Robot scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

# Optional viewer-only camera tracking. False preserves the stock free camera.
CAMERA_FOLLOW = False
CAMERA_FOLLOW_BODY = ""  # empty = pelvis for G1, base_link otherwise
CAMERA_LOOKAT_OFFSET = (0.0, 0.0, 0.0)
CAMERA_FOLLOW_YAW = False  # False keeps a world-fixed heading
CAMERA_SMOOTHING_TAU = 0.15  # seconds; 0 disables damping
CAMERA_DISTANCE = 3.0
CAMERA_AZIMUTH = 90.0
CAMERA_ELEVATION = -20.0
