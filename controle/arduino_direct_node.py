#!/usr/bin/env python3
"""Caméra -> modèle -> Arduino : aucune dépendance ROS, aucun topic."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,csv,hashlib,json,threading,time,uuid
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from real_image_geometry import canonical_corners,WIDTH,HEIGHT,K_SIM
from distilled_image_policy import Policy
from imitation_runtime import Runtime
from arduino_serial_link import ArduinoLink,encode_command

def main():
    p=argparse.ArgumentParser(description=__doc__);here=Path(__file__).resolve().parent
    p.add_argument('--calibration',type=Path,required=True);p.add_argument('--model',type=Path,default=here/'modele.pt')
    p.add_argument('--camera',default='/dev/video0');p.add_argument('--port',default='/dev/ttyACM0')
    p.add_argument('--focus',type=float,default=0.);p.add_argument('--linear-sign',type=int,choices=[-1,1],default=-1);p.add_argument('--angular-sign',type=int,choices=[-1,1],default=-1)
    p.add_argument('--execute',action='store_true');p.add_argument('--duration',type=float,default=10.);p.add_argument('--no-display',action='store_true')
    p.add_argument('--output-root',type=Path,default=here/'data/arduino_direct')
    a=p.parse_args()
    if not 0<a.duration<=600 or not np.isfinite(a.focus):p.error('Duree 0..600 s et focus fini requis')
    import cv2,yaml
    cv2.setNumThreads(1)
    config=yaml.safe_load(a.calibration.read_text());k=np.asarray(config['camera_matrix'],float);d=np.asarray(config['dist_coeff'],float);width,height=int(config['image_width']),int(config['image_height'])
    if k.shape!=(3,3) or not np.isfinite(k).all() or not np.isfinite(d).all():p.error('Calibration invalide')
    policy=Policy.load(a.model);runtime=Runtime(policy)
    parameters=cv2.aruco.DetectorParameters();parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100),parameters)
    folder=a.output_root/('essai_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+uuid.uuid4().hex[:6]);folder.mkdir(parents=True,exist_ok=False)
    cap=None;link=None;thread=None;stop=threading.Event();lock=threading.Lock();latest=[None];failure=[None];last=None;points=None;reason='ATTENTE CAMERA';count=0
    file=(folder/'commandes.csv').open('x',newline='');writer=csv.writer(file);writer.writerow(['monotonic_ns','image_stamp_ns','phase','v_prediction','omega_prediction','v_envoyee_modele','omega_envoyee_modele','ligne_serie']+[f'corner_{i}_{axis}_canonical_px' for i in range(4) for axis in ('u','v')])
    serial_log=(folder/'serie.jsonl').open('x')
    (folder/'configuration.json').write_text(json.dumps({'execute':a.execute,'port':a.port,'baud':57600,'camera':a.camera,'calibration':str(a.calibration),'calibration_sha256':hashlib.sha256(a.calibration.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256(a.model.read_bytes()).hexdigest(),'signs':[a.linear_sign,a.angular_sign],'limits':{'v':[-.02,.03],'omega':[-.08,.08]},'duration_s':a.duration,'source':'detected corners and ten commands sent, no odom or ROS','canonical_K':K_SIM.tolist(),'watchdog':'host thread, .3 s; NOT firmware timeout'},indent=2))
    try:
        cap=cv2.VideoCapture(a.camera,cv2.CAP_V4L2)
        if not cap.isOpened():raise RuntimeError('Camera inaccessible : '+a.camera)
        for prop,value in [(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG')),(cv2.CAP_PROP_FRAME_WIDTH,width),(cv2.CAP_PROP_FRAME_HEIGHT,height),(cv2.CAP_PROP_FPS,15),(cv2.CAP_PROP_BUFFERSIZE,1),(cv2.CAP_PROP_AUTOFOCUS,0),(cv2.CAP_PROP_FOCUS,a.focus)]:cap.set(prop,value)
        if a.execute and (cap.get(cv2.CAP_PROP_AUTOFOCUS)!=0 or abs(cap.get(cv2.CAP_PROP_FOCUS)-a.focus)>1):raise RuntimeError('Focus non confirme')
        def capture():
            try:
                while not stop.is_set():
                    ok,frame=cap.read();stamp=time.monotonic_ns()
                    if not ok:failure[0]='Lecture camera impossible';return
                    if frame.shape[:2]!=(height,width):failure[0]='Resolution differente du YAML';return
                    with lock:latest[0]=(stamp,frame)
            except Exception as exc:failure[0]=str(exc)
        thread=threading.Thread(target=capture,daemon=True);thread.start()
        end=time.monotonic()+5
        while latest[0] is None and failure[0] is None and time.monotonic()<end:time.sleep(.02)
        if failure[0] or latest[0] is None:raise RuntimeError(failure[0] or 'Aucune image recue')
        if a.execute:
            link=ArduinoLink(a.port,serial_log,a.linear_sign,a.angular_sign)
            print('SERIE OUVERTE : '+a.port+' @57600 ; MOTEURS ACTIFS ; signes '+str((a.linear_sign,a.angular_sign)),flush=True)
        else:print('APERCU : port serie non ouvert, aucun moteur commande',flush=True)
        print('Resultats : '+str(folder),flush=True)
        started=time.monotonic();tick=started
        while time.monotonic()-started<a.duration:
            now=time.monotonic_ns();pred=(0.,0.);actual=(0.,0.);frame=None
            if failure[0]:raise RuntimeError(failure[0])
            if link is not None and link.failure:raise RuntimeError('Erreur serie : '+link.failure)
            if link is not None and link.stale:runtime=Runtime(policy);last=None
            with lock:item=latest[0]
            if item is not None and 0<=(now-item[0])*1e-9<=.3:
                stamp,original=item;frame=original.copy()
                if stamp!=last:
                    corners,ids,_=detector.detectMarkers(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY));matches=[] if ids is None else np.flatnonzero(ids.flatten()==23);points=None
                    if len(matches)==1:
                        raw=corners[int(matches[0])].reshape(4,2);points=canonical_corners(raw,k,d);cv2.aruco.drawDetectedMarkers(frame,[raw.reshape(1,4,2)],np.array([[23]],np.int32))
                    runtime.observe(points,stamp,WIDTH,HEIGHT);last=stamp
                pred=runtime.command(time.monotonic_ns());reason='OBJECTIF IMAGE' if policy.finished else runtime.reason
                if link is not None:actual=(float(np.clip(pred[0],-.02,.03)),float(np.clip(pred[1],-.08,.08)))
            else:reason='PAUSE : CAMERA ANCIENNE'
            if item is None or (time.monotonic_ns()-item[0])*1e-9>.3:actual=(0.,0.);reason='PAUSE : IMAGE ANCIENNE APRES CALCUL'
            line=''
            if link is not None:
                link.send(*actual);line=encode_command(*actual,a.linear_sign,a.angular_sign).decode().strip()
            runtime.record_sent(time.monotonic_ns(),*actual);count+=1
            writer.writerow([now,last,reason,*pred,*actual,line]+(points.reshape(8).tolist() if points is not None else ['']*8));file.flush()
            if frame is not None and not a.no_display:
                preview=cv2.resize(frame,(960,540));cv2.putText(preview,('ARDUINO DIRECT' if a.execute else 'APERCU')+' | '+reason,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.putText(preview,f'prediction {pred[0]:+.3f},{pred[1]:+.3f} | serie {line or "NON OUVERTE"}',(10,50),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.imshow('Camera -> modele -> Arduino',preview)
                if cv2.waitKey(1)&255 in (27,ord('q')):reason='ARRET UTILISATEUR';break
                if cv2.getWindowProperty('Camera -> modele -> Arduino',cv2.WND_PROP_VISIBLE)<1:reason='FENETRE FERMEE';break
            tick+=.1;time.sleep(max(0,min(.1,tick-time.monotonic())))
    except KeyboardInterrupt:reason='CTRL+C'
    finally:
        # Toujours arrêter avant le nettoyage de caméra et des fichiers.
        try:
            if link is not None:link.close()
        finally:
            stop.set()
            if thread is not None:thread.join(timeout=1)
            if cap is not None and (thread is None or not thread.is_alive()):cap.release()
            file.close();serial_log.close();cv2.destroyAllWindows()
            (folder/'resume.json').write_text(json.dumps({'phase_finale':reason,'ticks':count,'execute':a.execute,'hardware_success':'not established; serial writes are not proof of motor motion'},indent=2))
if __name__=='__main__':main()
