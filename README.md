Based on https://github.com/thomasweng15/vive-mujoco.

# New Features
- Implement 3 mode of simulation type in Mujoco : record, playback, regular
- Able to collect data in virtual world and reload the data
- Optimize the Grippers and Contacts(shape, skin, parameters, solvers and so on)
- Make the grasping task more smoothly on Mujoco Engine

# Nots
- mjvive.py currently got a black screen issue, use mjvive.cpp instead

# Todo
- [ ] Generating expert data from simulation to be used in the GAIL/bc
- [ ] Save states, observations, rewards and other required infomaitions
- [ ] Transform the saved contents into baselines_VRIL format(NPZ files)

# vive-mujoco

## Setup

1. Install Mujoco 2.0 `mujoco200` into the `.mujoco/` directory and get a license key
2. Clone this repo into the `.mujoco/` directory
3. Get dependencies: put header, dll, and lib files into directories in `mujoco200/` as follows:
    * `bin/`
        * `glew32.dll`
        * `glew32.lib`
        * `openvr_api.dll`
        * `openvr_api.lib`
    * `include/`
        * `GL/glew.h` (make a new folder named GL)
        * `openvr.h`
4. Run makefile in `src/` to build `.exe` files in the `mujoco200/bin/` folder
    * Open VS2015 x64 Native Tools Command Prompt
    * Navigate to `src/` directory in repo
    * Run `nmake -f makefile`
5. Run the code after making by navigating to `mujoco200/bin/` and running `mjvive.exe ..\..\vive-mujoco\model\sawyer_rope.xml`->`mjvive.exe ..\..\mujoco-htcvive\model\sawyer_rope.xml`

## Notes

* `mjvive.py` currently doesn't work
* `minivive.cpp` is `mjvive.cpp` without controllers

## Running the code

Instructions for starting the Vive
* Make sure the Vive is plugged in and turned on. Turn on the controllers.
* Move the headset and controllers to the play space where the base stations can track them.
* Run the SteamVR app and check that the Vive and controllers are tracking with the base stations. 
* The Vive is now ready. To turn it off, close the apps and turn the Vive off.

Instructions for running the Vive + MuJoCo demo
* The Vive and at least one controller need to be on and tracking.
* Run the Anaconda Prompt as an administrator (right click and select run as admin)
* Navigate to `${HOME}\.mujoco\mjpro200\bin`
* Run `mjvive.exe`. Type in a MuJoCo model file when prompted, for example: `..\model\humanoid.xml`
* The view from the headset will appear in a window on the screen. You can now use the Vive to interact with the MuJoCo model.
