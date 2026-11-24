#!/usr/bin/env python3
"""Read only retained original ULogs; no device/network/parameter changes."""
import hashlib,json,sys,subprocess
from pathlib import Path
import numpy as np
from pyulog import ULog
REPO=Path('/home/jikl62/FC01_OrangePi_Navigation_20261028')
INV=REPO/'docs/investigations/20261116_stage1/evidence/ULOG_INVENTORY.json'
SUMMARY=REPO/'docs/investigations/20261116_stage1/evidence/ulog/summary.json'
inventory=json.loads(INV.read_text())
filters=['vehicle_status','vehicle_land_detected','vehicle_local_position','vehicle_attitude','estimator_status_flags','estimator_selector_status','estimator_aid_src_optical_flow','estimator_aid_src_rng_hgt','estimator_aid_src_ev_pos','estimator_aid_src_ev_yaw','estimator_aid_src_baro_hgt','estimator_aid_src_gnss_hgt','sensor_optical_flow','vehicle_optical_flow','distance_sensor','vehicle_visual_odometry','vehicle_odometry','vehicle_angular_velocity']
params=['EKF2_HGT_REF','EKF2_GPS_CTRL','EKF2_BARO_CTRL','EKF2_EV_CTRL','EKF2_RNG_CTRL','EKF2_OF_CTRL','EKF2_OF_DELAY','EKF2_RNG_DELAY','EKF2_RNG_A_HMAX','EKF2_RNG_A_VMAX','EKF2_OF_POS_X','EKF2_OF_POS_Y','EKF2_OF_POS_Z','EKF2_RNG_POS_X','EKF2_RNG_POS_Y','EKF2_RNG_POS_Z','SYS_HAS_GPS','SYS_HAS_BARO','SYS_HAS_NUM_DIST','SYS_HAS_NUM_OF','SENS_FLOW_ROT','SENS_TFMINI_CFG','SENS_EN_PAA3905','SENS_EN_PAW3902','MPC_ALT_MODE','COM_OF_LOSS_T','COM_OBL_RC_ACT','UXRCE_DDS_CFG','UXRCE_DDS_DOM_ID']
flags=['cs_opt_flow','cs_rng_hgt','cs_baro_hgt','cs_gps_hgt','cs_gnss_pos','cs_ev_pos','cs_ev_vel','cs_ev_hgt','cs_ev_yaw','cs_in_air','cs_rng_terrain','cs_opt_flow_terrain','cs_rng_fault','cs_rng_stuck','cs_inertial_dead_reckoning']
reset_fields=['xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter','dist_bottom_reset_counter']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def counts(a):
 v,c=np.unique(a,return_counts=True)
 return {str(x.item()):int(y) for x,y in zip(v,c)}
def stats(a):
 a=np.asarray(a);f=a[np.isfinite(a)]
 return {'samples':int(a.size),'finite':int(f.size),'min':float(f.min()) if len(f) else None,'max':float(f.max()) if len(f) else None,'mean':float(f.mean()) if len(f) else None}
def dataset_summary(ds,start=None,end=None):
 x=ds.data;t=x.get('timestamp',np.array([]));mask=np.ones(len(t),bool) if start is None else ((t>=start)&(t<end))
 out={'topic':ds.name,'instance':ds.multi_id,'samples':int(mask.sum())}
 if mask.any():out['first_us']=int(t[mask][0]);out['last_us']=int(t[mask][-1])
 if ds.name=='vehicle_local_position':
  out['numeric']={k:stats(x[k][mask]) for k in ['x','y','z','vx','vy','vz','z_deriv','dist_bottom'] if k in x}
  out['validity_counts']={k:counts(x[k][mask]) for k in ['xy_valid','z_valid','v_xy_valid','v_z_valid','dist_bottom_valid','dead_reckoning','heading_good_for_control'] if k in x}
  out['reset_counts']={k:counts(x[k][mask]) for k in reset_fields if k in x}
  out['reset_transitions']={k:[{'timestamp_us':int(t[i]),'from':int(x[k][i-1]),'to':int(x[k][i])} for i in range(1,len(t)) if mask[i] and mask[i-1] and x[k][i]!=x[k][i-1]] for k in reset_fields if k in x}
 elif ds.name=='vehicle_attitude':out['quat_reset_counts']=counts(x['quat_reset_counter'][mask]) if 'quat_reset_counter' in x else None
 elif ds.name=='estimator_status_flags':out['fusion_flag_counts']={k:counts(x[k][mask]) for k in flags if k in x}
 elif ds.name=='estimator_selector_status':out['primary_instance_counts']=counts(x['primary_instance'][mask]) if 'primary_instance' in x else None
 elif ds.name=='vehicle_land_detected':out['land_counts']={k:counts(x[k][mask]) for k in ['landed','ground_contact','maybe_landed','at_rest'] if k in x}
 else:out['reported_validity']={k:counts(x[k][mask]) for k in ['fused','innovation_rejected','quality','orientation','min_distance','max_distance'] if k in x}
 return out
records=[]
retained=list(inventory['files'])
extra_path=Path('/home/jikl62/桌面/log_82_UnknownDate.ulg')
if extra_path.is_file() and sha(extra_path) not in {x['sha256'] for x in retained}:
 retained.append({'sha256':sha(extra_path),'size':extra_path.stat().st_size,'paths':[str(extra_path)]})
for n,item in enumerate(retained):
 candidates=[Path(p) for p in item['paths'] if Path(p).is_file()]
 if not candidates:
  records.append({'expected_sha256':item['sha256'],'error':'no retained path still exists','paths':item['paths']});continue
 p=candidates[0];digest=sha(p);assert digest==item['sha256'],(p,digest)
 u=ULog(str(p),message_name_filter_list=filters)
 modes=[d for d in u.data_list if d.name=='vehicle_status']
 navset=sorted({int(v) for d in modes for v in d.data['nav_state']})
 positions=[]
 for d in modes:
  ts=d.data['timestamp'];nav=d.data['nav_state'];idx=np.flatnonzero(np.r_[True,nav[1:]!=nav[:-1]])
  for i,k in enumerate(idx):
   if nav[k]!=2:continue
   end=int(ts[idx[i+1]]) if i+1<len(idx) else int(u.last_timestamp+1)
   positions.append({'start_us':int(ts[k]),'end_exclusive_us':end,'duration_s':(end-int(ts[k]))/1e6,'status_samples':int(((ts>=ts[k])&(ts<end)).sum()),'arming_counts':counts(d.data['arming_state'][(ts>=ts[k])&(ts<end)]),'failsafe_counts':counts(d.data['failsafe'][(ts>=ts[k])&(ts<end)]),'topics':[dataset_summary(x,int(ts[k]),end) for x in u.data_list if x.name!='vehicle_status']})
 info={k:u.msg_info_dict.get(k) for k in ['ver_sw','ver_hw','ver_sw_release','sys_os_name','sys_name','sys_uuid']}
 physical=info['ver_hw']=='DAMIAO_DM_FC01' and info['sys_os_name']=='NuttX'
 rec={'source':str(p),'sha256':digest,'bytes':p.stat().st_size,'identity':info,'is_physical_fc01_metadata':physical,'selected_parse_start_us':int(u.start_timestamp),'selected_parse_last_us':int(u.last_timestamp),'nav_states':navset,'initial_parameters':{k:u.initial_parameters.get(k) for k in params},'parameter_changes':[[int(ts),name,val] for ts,name,val in u.changed_parameters if name in params],'topics':[dataset_summary(d) for d in u.data_list if d.name!='vehicle_status'],'position_intervals':positions,'source_unchanged_after_parse':sha(p)==digest}
 records.append(rec);print(n+1,p.name,info['ver_hw'],navset,'Position intervals',len(positions),flush=True)
px=Path('/home/jikl62/fc01_px4_dds_build/PX4-DM-FC01-v1.16')
source_files=['src/modules/ekf2/EKF2.cpp','src/modules/ekf2/module.yaml','src/modules/ekf2/params_optical_flow.yaml','src/modules/ekf2/params_range_finder.yaml','src/modules/uxrce_dds_client/dds_topics.yaml']
new_record=next((r for r in records if r.get('source')==str(extra_path)),None)
if new_record is not None:
 u=ULog(str(extra_path))
 new_record['full_parse_start_us']=int(u.start_timestamp);new_record['full_parse_last_us']=int(u.last_timestamp);new_record['full_parse_duration_s']=(u.last_timestamp-u.start_timestamp)/1e6
 new_record['all_initial_parameters']=u.initial_parameters
 new_record['full_logged_messages']=[{'timestamp_us':m.timestamp,'log_level':m.log_level,'message':m.message} for m in u.logged_messages]
 new_record['topic_raw_samples']={f'{d.name}/{d.multi_id}':{k:[x.item() if isinstance(x,np.generic) else x for x in v] for k,v in d.data.items()} for d in u.data_list if d.name in ['vehicle_status','vehicle_land_detected','estimator_status_flags','distance_sensor','estimator_aid_src_rng_hgt','estimator_aid_src_baro_hgt','estimator_aid_src_ev_pos','estimator_aid_src_ev_yaw']}
 new_record['all_topic_counts']=[{'name':d.name,'instance':d.multi_id,'samples':len(d.data.get('timestamp',[]))} for d in u.data_list]
 new_record['dropout_count']=len(u.dropouts)
 status=new_record['topic_raw_samples']['vehicle_status/0']
 armed_start=status['timestamp'][0];disarm=next(t for t,a in zip(status['timestamp'],status['arming_state']) if a!=2)
 new_record['armed_position_interval_observed']={'start_us':armed_start,'end_exclusive_us':disarm,'duration_s':(disarm-armed_start)/1e6,'note':'Observed held status interval, not independently proven liftoff; first status predates ULog header.'}
report={'audit_date_local':'2026-11-18','scope':'Direct read of prior retained unique-hash inventory plus one newly discovered direct desktop ULog. Not current board; not a whole-filesystem scan; independent recordings. Filtered topic parse timestamps are not full-log durations.','reproduction':'/usr/bin/python3 /tmp/fc01-step2-ekf-audit.py','inventory':{'path':str(INV),'sha256':sha(INV),'retained_unique_hashes':len(inventory['files']),'new_direct_path_discovery':['/home/jikl62/桌面/log_82_UnknownDate.ulg'],'discovery_command':"rg --files /home/jikl62/DroneProjects/05_飞行日志与测试证据 /home/jikl62/桌面 /home/jikl62/Desktop 2>/dev/null | rg '\\.ulg$'",'old_direct_summary_sha256':sha(SUMMARY)},'records':records,'counts':{'retained_records':len(records),'physical_fc01':sum(x.get('is_physical_fc01_metadata',False) for x in records),'records_with_position':sum(bool(x.get('position_intervals')) for x in records),'physical_fc01_with_position':sum(x.get('is_physical_fc01_metadata',False) and bool(x.get('position_intervals')) for x in records),'errors':sum('error' in x for x in records)},'px4_source':{'path':str(px),'commit':subprocess.check_output(['git','-C',str(px),'rev-parse','HEAD'],text=True).strip(),'dirty':subprocess.check_output(['git','-C',str(px),'status','--short'],text=True).splitlines(),'sha256':{s:sha(px/s) for s in source_files}},'reproduction_script_sha256':sha(Path(__file__)),'live_board_verified':False,'current_position_mtf_fusion_verified':False,'new_hardware_position_record_found':True,'missing_evidence':['Current hardware Position ULog and PX4 firmware identity','Current EKF2 parameter export and fusion flags','Current board runtime.yaml and fc01-navigation.service readback','Current DDS graph including actual angular-velocity publication','Camera exposure-time/transport-delay calibration and physical extrinsics']}
Path('/tmp/fc01-step2-ekf-baseline.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print('counts',report['counts'])
