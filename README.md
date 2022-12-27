# SUMMIT-XL


##### Do you want to start? 
##### No Problem. Follow me.

# Installation
1. Clone the repo
> git clone https://github.com/We2Am-BaSsem/SUMMIT-XL.git
2. After cloning is done run the `setupProject.sh` and tthis will do all the magic.
> source setupProject.sh

if you face issue such as:
```
RLException: unused args [arm_manufacturer, arm_model] for include of [/mnt/FC8006E780
06A7EA/${path to project}/src/summit_xl
_common/summit_xl_control/launch/summit_xl_control.launch]
The traceback for the exception was written to the log file
```
just remove lines 104 and 105 in `SUMMIT-XL/src/summit_xl_sim/summit_xl_gazebo/launch/summit_xl_one_robot.launch`

3. to launch Gazebo + rViz: run `run_rVis.sh`
> source run_rVis.sh
