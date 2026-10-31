"""Read-only, timestamped TF analysis of stationary bag landmarks."""
import argparse
import json
import sqlite3
from collections import defaultdict
from pathlib import Path
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer
from landmark_fit import fit_round, spatial_clusters


def rotation(q):
    a=np.array([q.x,q.y,q.z,q.w]);x,y,z,w=a/np.linalg.norm(a)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    data=defaultdict(list)
    with sqlite3.connect('file:'+str(args.bag.resolve())+'?mode=ro',uri=True) as db:
        topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
        for i,blob in db.execute('select topic_id,data from messages order by timestamp'):
            name,typ=topics[i]
            if name in ['/parrot1/scan/points','/parrot1/tf','/parrot1/tf_static','/parrot1/odometry']:
                data[name].append(deserialize_message(blob,typ))
    rclpy.init()
    buffer=Buffer(cache_time=Duration(seconds=120))
    for topic,static in [('/parrot1/tf_static',True),('/parrot1/tf',False)]:
        for message in data[topic]:
            for t in message.transforms:
                (buffer.set_transform_static if static else buffer.set_transform)(t,'offline')
    rows=[]; failures=[]; samples={}
    for index,msg in enumerate(data['/parrot1/scan/points']):
        try:t=buffer.lookup_transform('parrot1_odom',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
        except Exception as e:
            failures.append({'index':index,'reason':str(e)});continue
        records=point_cloud2.read_points(msg,field_names=['x','y','z'],skip_nans=False)
        p=np.stack([records[k] for k in ['x','y','z']],axis=-1).reshape(-1,3).astype(float)
        p=p[np.isfinite(p).all(axis=1)]@rotation(t.rotation).T+[t.translation.x,t.translation.y,t.translation.z]
        for name,xy,zlimits,dim in [('tree',[4,-4],[.2,.9],2),('animal',[-4,4],[.15,.60],3)]:
            mask=(abs(p[:,0]-xy[0])<1)&(abs(p[:,1]-xy[1])<1)&(p[:,2]>=zlimits[0])&(p[:,2]<=zlimits[1])
            # Exclude the known +X nose half-space; this does not fix fitted centre/radius.
            if name=='animal':mask &= p[:,0]<-3.8
            selected=p[mask]
            row={'index':index,'name':name,'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,'selected_points':len(selected)}
            try:
                cluster_results=[]; supported=[]
                for cluster in spatial_clusters(selected):
                    try:
                        candidate=fit_round(cluster[:,:dim])
                        candidate['status']='FIT_CANDIDATE' if candidate['p95_residual_m']<=.05 else 'REJECTED_HIGH_RESIDUAL'
                        if candidate['status']=='FIT_CANDIDATE':supported.append(candidate)
                        cluster_results.append(candidate)
                    except ValueError as e:
                        cluster_results.append({'points':len(cluster),'status':'INSUFFICIENT_SUPPORT','reason':str(e)})
                row['clusters']=cluster_results
                if len(supported)!=1:
                    raise ValueError(f'INSUFFICIENT_OR_AMBIGUOUS_SUPPORT: {len(supported)} supported clusters')
                fit=supported[0]
                row.update(fit);row['status']='FIT_CANDIDATE'
                row['xy_error_to_nominal_m']=float(np.linalg.norm(np.array(fit['centre'][:2])-xy))
                if fit['p95_residual_m']>.05:row['status']='REJECTED_HIGH_RESIDUAL'
            except ValueError as e:row.update(status='INSUFFICIENT_SUPPORT',reason=str(e))
            rows.append(row)
            if name not in samples:samples[name]=selected
    report={'status':'OFFLINE_CANDIDATES_NOT_ACCEPTANCE','bag':str(args.bag), 'clouds':len(data['/parrot1/scan/points']), 'tf_failures':failures, 'targets':{},'limitations':['Reference geometry is from current large_demo; bag has no historical Gazebo entity poses, so historical landmark identity is conditional.', 'Fits are free centre/radius algebraic fits; ROI and nose exclusion use nominal geometry.', 'Per-frame repeatability is not independent mapping accuracy; simulated absolute odometry is used.', '60 degree arc and 0.05 m residual are diagnostic gates, not team acceptance limits.']}
    for name in ['tree','animal']:
        allrows=[r for r in rows if r['name']==name];good=[r for r in allrows if r['status']=='FIT_CANDIDATE']
        summary={'frames':len(allrows),'supported_fits':len(good),'selected_points_range':[min(r['selected_points'] for r in allrows),max(r['selected_points'] for r in allrows)]} if allrows else {'frames':0,'supported_fits':0}
        if good:
            centres=np.array([r['centre'] for r in good]);summary.update(median_centre= np.median(centres,axis=0).tolist(),centre_peak_to_peak_m=np.ptp(centres,axis=0).tolist(),median_radius_m=float(np.median([r['radius'] for r in good])),median_xy_error_m=float(np.median([r['xy_error_to_nominal_m'] for r in good])),median_p95_residual_m=float(np.median([r['p95_residual_m'] for r in good])))
        report['targets'][name]=summary
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    (args.output/'frames.json').write_text(json.dumps(rows,indent=2))
    np.savez_compressed(args.output/'first_frame_candidates.npz',**samples)
    print(json.dumps(report,indent=2))
    rclpy.shutdown()


if __name__=='__main__':main()
