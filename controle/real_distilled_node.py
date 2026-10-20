#!/usr/bin/env python3
"""Essai matériel expérimental, caméra USB et ROS 2 Humble ; aperçu par défaut."""
import os
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'): os.environ[name]='1'
import argparse, csv, json, threading, time, uuid
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from real_image_geometry import canonical_corners, WIDTH, HEIGHT

def main():
    p=argparse.ArgumentParser(description=__doc__)
    sim=Path(__file__).resolve().parent
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--model',type=Path,default=sim/'modele.pt')
    p.add_argument('--camera',default='/dev/video2');p.add_argument('--focus',type=float,default=0.)
    p.add_argument('--cmd-topic',default='/cmd_vel');p.add_argument('--execute',action='store_true')
    p.add_argument('--no-display',action='store_true');p.add_argument('--duration',type=float,default=120.)
    args=p.parse_args()
    import cv2, yaml, rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from distilled_image_policy import Policy
    from imitation_runtime import Runtime
    if not 0<args.duration<=600: p.error('Durée requise : 0 < durée <= 600 s')
    config=yaml.safe_load(args.calibration.read_text())
    width,height=int(config['image_width']),int(config['image_height'])
    matrix=np.asarray(config['camera_matrix'],float);distortion=np.asarray(config['dist_coeff'],float)
    if matrix.shape!=(3,3) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():p.error('Calibration invalide')
    cv2.setNumThreads(1)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters=cv2.aruco.DetectorParameters();parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    detector=cv2.aruco.ArucoDetector(dictionary,parameters)
    policy=Policy.load(args.model);runtime=Runtime(policy)
    cap=cv2.VideoCapture(args.camera,cv2.CAP_V4L2)
    if not cap.isOpened():raise RuntimeError('Caméra inaccessible : '+args.camera)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,width);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
    cap.set(cv2.CAP_PROP_FPS,15);cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS,0);cap.set(cv2.CAP_PROP_FOCUS,args.focus)
    if args.execute and (cap.get(cv2.CAP_PROP_AUTOFOCUS)!=0 or abs(cap.get(cv2.CAP_PROP_FOCUS)-args.focus)>1):
        cap.release();raise RuntimeError('Focus non confirmé par le pilote')
    lock=threading.Lock();stop=threading.Event();latest=[None];failure=[None]
    def capture():
        try:
            while not stop.is_set():
                ok,frame=cap.read();stamp=time.monotonic_ns()
                if not ok:failure[0]='Lecture caméra impossible';break
                if frame.shape[:2]!=(height,width):failure[0]='Résolution différente du YAML';break
                with lock:latest[0]=(stamp,frame)
        except Exception as exc:failure[0]=str(exc)
    thread=threading.Thread(target=capture,daemon=True);thread.start()
    folder=sim/'data/real_rollouts'/('essai_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+uuid.uuid4().hex[:6]);folder.mkdir(parents=True,exist_ok=False)
    (folder/'configuration.json').write_text(json.dumps(dict(vars(args),calibration=str(args.calibration),model=str(args.model),max_v=.03,max_omega=.08,canonical_K=__import__('real_image_geometry').K_SIM.tolist()),default=str,indent=2))
    file=(folder/'commandes.csv').open('x',newline='');writer=csv.writer(file)
    writer.writerow(['monotonic_ns','image_monotonic_ns','phase','v_pred','omega_pred','v_sent','omega_sent']+[f'corner_{i}_{axis}' for i in range(4) for axis in ('u','v')])
    rclpy.init()
    class Robot(Node):
        def __init__(self):
            super().__init__('essai_modele_image_reel');self.pub=self.create_publisher(Twist,args.cmd_topic,1)
            self.last=None;self.points=None;self.started=time.monotonic();self.done=False
            self.create_timer(.1,self.tick)
            self.get_logger().info(('MOTEURS ACTIFS' if args.execute else 'APERCU : aucune commande publiée')+' ; résultats : '+str(folder))
        def send(self,v=0.,w=0.):
            if args.execute:
                msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.pub.publish(msg)
        def tick(self):
            now=time.monotonic_ns();pred=(0.,0.);actual=(0.,0.);frame=None
            with lock:item=latest[0]
            fresh=item is not None and 0<=(now-item[0])*1e-9<=.3
            if failure[0]:self.done=True
            if time.monotonic()-self.started>args.duration:self.done=True
            if fresh and not self.done:
                stamp,frame=item
                if stamp!=self.last:
                    corners,ids,_=detector.detectMarkers(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
                    matches=[] if ids is None else np.flatnonzero(ids.flatten()==23)
                    self.points=None
                    if len(matches)==1:
                        raw=corners[int(matches[0])].reshape(4,2)
                        self.points=canonical_corners(raw,matrix,distortion)
                        cv2.aruco.drawDetectedMarkers(frame,[raw.reshape(1,4,2)],np.array([[23]],np.int32))
                    runtime.observe(self.points,stamp,WIDTH,HEIGHT);self.last=stamp
                pred=runtime.command(now)
                if args.execute:actual=(float(np.clip(pred[0],-.02,.03)),float(np.clip(pred[1],-.08,.08)))
            else:runtime.reason='ARRET' if self.done else 'PAUSE CAMERA ANCIENNE/ABSENTE'
            if item is None or (time.monotonic_ns()-item[0])*1e-9>.3:
                actual=(0.,0.);runtime.reason="PAUSE : TRAITEMENT TROP ANCIEN"
            self.send(*actual);runtime.record_sent(time.monotonic_ns(),*actual)
            writer.writerow([now,self.last,runtime.reason,*pred,*actual]+(self.points.reshape(8).tolist() if self.points is not None else ['']*8));file.flush()
            if frame is not None and not args.no_display:
                preview=cv2.resize(frame,(960,540));cv2.putText(preview,('EXECUTION' if args.execute else 'APERCU')+' '+runtime.reason,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.putText(preview,f'pred v={pred[0]:+.3f} w={pred[1]:+.3f} ; envoyee v={actual[0]:+.3f} w={actual[1]:+.3f}',(10,50),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.imshow('Modele camera reelle',preview)
                if cv2.waitKey(1)&255 in (27,ord('q')):self.done=True;self.send()
            if self.done:raise KeyboardInterrupt
    node=Robot()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            for _ in range(3):node.send();time.sleep(.03)
        stop.set();thread.join(timeout=1)
        if not thread.is_alive():cap.release()
        file.close();cv2.destroyAllWindows();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if failure[0]:print('Arrêt : '+failure[0])
if __name__=='__main__':main()
