"""Conservative, reversible ideal spring-follower study; no acoustic claims."""
import numpy as np


def detent(angle_deg,preload_N=4,stiffness_N_per_mm=2,depth_mm=.6,halfwidth_deg=6):
    a=np.asarray(angle_deg,float)
    delta=np.where(abs(a)<=abs(a-48),a,a-48)
    w=np.deg2rad(halfwidth_deg);q=np.deg2rad(delta)
    inside=abs(q)<w
    lift=np.where(inside,depth_mm*(1-np.cos(np.pi*q/w))/2,depth_mm)
    slope=np.where(inside,depth_mm*np.pi/(2*w)*np.sin(np.pi*q/w),0)
    force=preload_N+stiffness_N_per_mm*lift
    energy=(preload_N*lift+.5*stiffness_N_per_mm*lift**2)/1000
    return dict(lift_mm=lift,force_N=force,energy_J=energy,cam_resisting_torque_Nm=force*slope/1000)


def cam_outline(angle_deg=0):
    # Cam recesses fixed at rotor angles90 and42: fixed follower at world90
    # encounters the two recesses at output angles0 and48 degrees.
    a=np.linspace(0,360,721)[:-1]
    distances=np.minimum(abs((a-90+180)%360-180),abs((a-42+180)%360-180))
    lift=detent(distances)['lift_mm']
    r=45-.6+lift
    phi=np.deg2rad(a+angle_deg)
    return np.stack([r*np.cos(phi),r*np.sin(phi)],axis=-1)


def report():
    angles=np.linspace(0,48,481);v=detent(angles)
    mean_r=2/3*(22**3-10**3)/(22**2-10**2)/1000
    return dict(detent_type='metal ball plunger + two cam recesses',output_positions_deg=[0,48],
                cam_radius_mm=45,groove_depth_mm=.6,groove_halfwidth_deg=6,
                assumed_preload_N=4,assumed_stiffness_N_per_mm=2,
                maximum_cam_torque_Nm=float(abs(v['cam_resisting_torque_Nm']).max()),
                energy_barrier_J=float(v['energy_J'].max()),
                acoustic_click_verified=False,
                clutch=dict(type='bidirectional friction clutch upstream of input20',
                    assumed_friction_range=[.1,.3],preload_range_N=[20,200],
                    friction_faces=2,annulus_radii_mm=[10,22],mean_radius_mm=mean_r*1000,
                    illustrative_torque_range_Nm=[2*.1*20*mean_r,2*.3*200*mean_r],
                    safe_setting_Nm=None,
                    calibration_rule='normal+detent < slip < weakest admissible input; single jam may receive entire branch torque'),
                limitations=['1D radial follower energy; finite ball geometry, friction and sound are not certified',
                             'Buffer elastomer, spring data and friction are assumed; no material/load release'])
