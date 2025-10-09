import time
import sys
import openvr
import math
import yaml


import numpy as np

from functools import lru_cache
from scipy.spatial.transform import Rotation as R

def convert_to_quaternion(pose_mat):
    vector_1 = pose_mat[0][:3]
    vector_2 = pose_mat[1][:3]
    vector_3 = pose_mat[2][:3]
    rot_matrix = np.array([vector_1, vector_2, vector_3])
    translation_vector = np.array([
        pose_mat[0][3],
        pose_mat[1][3],
        pose_mat[2][3]
    ])

    p_final = translation_vector

    rot_vive_to_world = R.from_euler('Y', 90, degrees=True) * R.from_euler('X', -90, degrees=True)
    rot_vive_to_world = rot_vive_to_world.as_matrix()

    rot_match_convention = R.from_euler('z', 90, degrees=True) * R.from_euler('X', 90, degrees=True)
    rot_match_convention = rot_match_convention.as_matrix()

    rot_matrix = rot_match_convention @ ( rot_matrix @ rot_vive_to_world)
    p_final = p_final @ (rot_vive_to_world @ rot_match_convention)

    # Rot 180 degrees with respect the local x axis
    rot_180_x = R.from_euler('X', 180, degrees=True).as_matrix()
    rot_matrix = rot_matrix @ rot_180_x

    qx, qy, qz, qw = R.from_matrix(rot_matrix).as_quat()

    return [-p_final[2], -p_final[0], p_final[1], qx, qy, qz, qw]


def convert_to_euler(pose_mat):
    yaw = 180 / math.pi * math.atan2(pose_mat[1][0], pose_mat[0][0])
    pitch = 180 / math.pi * math.atan2(pose_mat[2][0], pose_mat[0][0])
    roll = 180 / math.pi * math.atan2(pose_mat[2][1], pose_mat[2][2])
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [x,y,z,yaw,pitch,roll]

#Define a class to make it easy to append pose matricies and convert to both Euler and Quaternion for plotting
class pose_sample_buffer():
    def __init__(self):
        self.i = 0
        self.index = []
        self.time = []
        self.x = []
        self.y = []
        self.z = []
        self.yaw = []
        self.pitch = []
        self.roll = []
        self.r_w = []
        self.r_x = []
        self.r_y = []
        self.r_z = []

    def append(self,pose_mat,t):
        self.time.append(t)
        self.x.append(pose_mat[0][3])
        self.y.append(pose_mat[1][3])
        self.z.append(pose_mat[2][3])
        self.yaw.append(180 / math.pi * math.atan(pose_mat[1][0] /pose_mat[0][0]))
        self.pitch.append(180 / math.pi * math.atan(-1 * pose_mat[2][0] / math.sqrt(pow(pose_mat[2][1], 2) + math.pow(pose_mat[2][2], 2))))
        self.roll.append(180 / math.pi * math.atan(pose_mat[2][1] /pose_mat[2][2]))
        r_w = math.sqrt(abs(1+pose_mat[0][0]+pose_mat[1][1]+pose_mat[2][2]))/2
        self.r_w.append(r_w)
        self.r_x.append((pose_mat[2][1]-pose_mat[1][2])/(4*r_w))
        self.r_y.append((pose_mat[0][2]-pose_mat[2][0])/(4*r_w))
        self.r_z.append((pose_mat[1][0]-pose_mat[0][1])/(4*r_w))

def get_pose(vr_obj):
    return vr_obj.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)


class vr_tracked_device():
    def __init__(self,vr_obj,index,device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj

    @lru_cache(maxsize=None)
    def get_serial(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_SerialNumber_String)

    def get_model(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModelNumber_String)

    def get_battery_percent(self):
        return self.vr.getFloatTrackedDeviceProperty(self.index, openvr.Prop_DeviceBatteryPercentage_Float)

    def is_charging(self):
        return self.vr.getBoolTrackedDeviceProperty(self.index, openvr.Prop_DeviceIsCharging_Bool)


    def sample(self,num_samples,sample_rate):
        interval = 1/sample_rate
        rtn = pose_sample_buffer()
        sample_start = time.time()
        for i in range(num_samples):
            start = time.time()
            pose = get_pose(self.vr)
            rtn.append(pose[self.index].mDeviceToAbsoluteTracking,time.time()-sample_start)
            sleep_time = interval- (time.time()-start)
            if sleep_time>0:
                time.sleep(sleep_time)
        return rtn
    
    def get_pose_quaternion(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return convert_to_quaternion(pose[self.index].mDeviceToAbsoluteTracking)
        else:
            return None

    def get_pose_euler(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
            print(pose)
        if pose[self.index].bPoseIsValid:
            return convert_to_euler(pose[self.index].mDeviceToAbsoluteTracking)
        else:
            return None

    def get_pose_matrix(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].mDeviceToAbsoluteTracking
        else:
            return None

    def get_velocity(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].vVelocity
        else:
            return None

    def get_angular_velocity(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].vAngularVelocity
        else:
            return None

    def controller_state_to_dict(self, pControllerState):
        # This function is graciously borrowed from https://gist.github.com/awesomebytes/75daab3adb62b331f21ecf3a03b3ab46
        # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState
        d = {}
        d['unPacketNum'] = pControllerState.unPacketNum
        d['trigger'] = pControllerState.rAxis[1].x # value = 0 if it is not pressed, 1 if it is pressed
        d['trackpad_x'] = pControllerState.rAxis[0].x # Goes from -1.0 to 1.0
        d['trackpad_y'] = pControllerState.rAxis[0].y # Goes from -1.0 to 1.0
        d['ulButtonPressed'] = pControllerState.ulButtonPressed # Always 0
        d['ulButtonTouched'] = pControllerState.ulButtonTouched # Always 0
        d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1) # 1 if pressed, 0 if not pressed
        d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1) # 1 if pressed, 0 if not pressed
        d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1) # 1 if touched, 0 if not touched
        d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1) # 1 if pressed, 0 if not pressed
        return d

    def get_controller_inputs(self):
        result, state = self.vr.getControllerState(self.index)
        return self.controller_state_to_dict(state)

    def trigger_haptic_pulse(self, duration_micros=1000, axis_id=0):
        """
        Causes devices with haptic feedback to vibrate for a short time.
        """
        self.vr.triggerHapticPulse(self.index ,axis_id, duration_micros)

class vr_tracking_reference(vr_tracked_device):
    def get_mode(self):
        return self.vr.getStringTrackedDeviceProperty(self.index,openvr.Prop_ModeLabel_String).decode('utf-8').upper()
    def sample(self,num_samples,sample_rate):
        print("Warning: Tracking References do not move, sample isn't much use...")

class triad_openvr():
    def __init__(self, configfile_path=None):
        # Initialize OpenVR in the
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.vrsystem = openvr.VRSystem()

        # Initializing object to hold indexes for various tracked objects
        self.object_names = {"Tracking Reference":[],"HMD":[],"Controller":[],"Tracker":[]}
        self.devices = {}
        self.device_index_map = {}
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                               openvr.k_unMaxTrackedDeviceCount)
 
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if poses[i].bDeviceIsConnected:
                self.add_tracked_device(i)

    def return_controller_serials(self):
        controller_serials = {}
        for device in self.object_names["Controller"]:
            controller_serials[self.devices[device].get_serial()] = device
        return controller_serials

    def __del__(self):
        openvr.shutdown()

    def get_pose(self):
        return get_pose(self.vr)

    def poll_vr_events(self):
        """
        Used to poll VR events and find any new tracked devices or ones that are no longer tracked.
        """
        event = openvr.VREvent_t()
        while self.vrsystem.pollNextEvent(event):
            if event.eventType == openvr.VREvent_TrackedDeviceActivated:
                self.add_tracked_device(event.trackedDeviceIndex)
            elif event.eventType == openvr.VREvent_TrackedDeviceDeactivated:
                if event.trackedDeviceIndex in self.device_index_map:
                    self.remove_tracked_device(event.trackedDeviceIndex)

    def wait_for_n_tracking_references(self, n=3, poll_interval=0.1):
        print(f"Waiting for {n} tracking references...")
        while len(self.object_names["Tracking Reference"]) < n:
            print(f"Found {len(self.object_names['Tracking Reference'])} tracking references. Waiting...")
            self.poll_vr_events()
            time.sleep(poll_interval)

    def reorder_tracking_references(self, desired_serial):
        references = self.object_names["Tracking Reference"]

        device_with_desired_serial = None
        for dev_name in references:
            if self.devices[dev_name].get_serial() == desired_serial:
                device_with_desired_serial = dev_name
                break
        
        if device_with_desired_serial:
            references.remove(device_with_desired_serial)
            references.insert(0, device_with_desired_serial)
        else:
            print(f"No device with serial {desired_serial} found in tracking references.")

    def reindex_tracking_references(self):
        references = self.object_names["Tracking Reference"]

        temp_names = []

        for i, dev_name in enumerate(references):
            tmp_name = f"temp_tr_ref_{i+1}"
            self.rename_device(dev_name, tmp_name)
            temp_names.append(tmp_name)

        for i, tmp_name in enumerate(temp_names):
            final_name = f"tracking_reference_{i+1}"
            self.rename_device(tmp_name, final_name)
            references[i] = final_name


    def add_tracked_device(self, tracked_device_index):
        i = tracked_device_index
        device_class = self.vr.getTrackedDeviceClass(i)

        if (device_class == openvr.TrackedDeviceClass_Controller):
            device_name = "controller_"+str(len(self.object_names["Controller"])+1)
            self.object_names["Controller"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"Controller")
            self.device_index_map[i] = device_name

        elif (device_class == openvr.TrackedDeviceClass_HMD):
            device_name = "hmd_"+str(len(self.object_names["HMD"])+1)
            self.object_names["HMD"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"HMD")
            self.device_index_map[i] = device_name

        elif (device_class == openvr.TrackedDeviceClass_GenericTracker):
            device_name = "tracker_"+str(len(self.object_names["Tracker"])+1)
            self.object_names["Tracker"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"Tracker")
            self.device_index_map[i] = device_name

        elif (device_class == openvr.TrackedDeviceClass_TrackingReference):
            device_name = "tracking_reference_"+str(len(self.object_names["Tracking Reference"])+1)
            self.object_names["Tracking Reference"].append(device_name)
            self.devices[device_name] = vr_tracking_reference(self.vr,i,"Tracking Reference")
            self.device_index_map[i] = device_name

    def remove_tracked_device(self, tracked_device_index):
        if tracked_device_index in self.device_index_map:
            device_name = self.device_index_map[tracked_device_index]
            self.object_names[self.devices[device_name].device_class].remove(device_name)
            del self.device_index_map[tracked_device_index]
            del self.devices[device_name]
        else:
            raise Exception("Tracked device index {} not valid. Not removing.".format(tracked_device_index))

    def rename_device(self,old_device_name,new_device_name):
        self.devices[new_device_name] = self.devices.pop(old_device_name)
        for i in range(len(self.object_names[self.devices[new_device_name].device_class])):
            if self.object_names[self.devices[new_device_name].device_class][i] == old_device_name:
                self.object_names[self.devices[new_device_name].device_class][i] = new_device_name

    def print_discovered_objects(self):
        for device_type in self.object_names:
            plural = device_type
            if len(self.object_names[device_type])!=1:
                plural+="s"
            print("Found "+str(len(self.object_names[device_type]))+" "+plural)
            for device in self.object_names[device_type]:
                if device_type == "Tracking Reference":
                    print("  "+device+" ("+self.devices[device].get_serial()+
                          ", Mode "+self.devices[device].get_model()+
                          ", "+self.devices[device].get_model()+
                          ")")
                else:
                    print("  "+device+" ("+self.devices[device].get_serial()+
                          ", "+self.devices[device].get_model()+")")
