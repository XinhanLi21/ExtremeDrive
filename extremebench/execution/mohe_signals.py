"""Measured-signal conversion, separate from road-wheel actuator parameters.
No torque decoding or brake reconstruction. Fitted offset is dataset-specific.
"""
import numpy as np

def normalize_frame(frame,calibration):
 def g(key):return frame['data_'+key].to_numpy(float)
 ratio=calibration['effective_steering_ratio'];offset=calibration['steering_wheel_zero_offset_deg']
 if not 5<ratio<40:raise ValueError('invalid candidate steering ratio')
 yaw=np.deg2rad(90-g('Heading_Z_GPS_rad_'));ve=g('GPS_VE');vn=g('GPS_VN')
 return dict(time_s=frame.iloc[:,0].to_numpy(float),gps_speed_mps=np.hypot(ve,vn),
     vx_gps_mps=ve*np.cos(yaw)+vn*np.sin(yaw),vy_gps_mps=-ve*np.sin(yaw)+vn*np.cos(yaw),
     yaw_rate_ccw_rad_s=np.deg2rad(g('Rate_RZ')),
     front_steer_candidate_rad=np.deg2rad(g('steerwheel_angle')-offset)/ratio,
     heading_enu_rad=yaw,
     **{k.lower()+'_speed_mps_candidate':g(k)/3.6 for k in ['Wheel_LF','Wheel_LR','Wheel_RF','Wheel_RR']})
