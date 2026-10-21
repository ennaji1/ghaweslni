"""Transport direct M<V>:<W>, indépendant de ROS et de la vision."""
import math,threading,time,json

def encode_command(v,w,linear_sign=-1,angular_sign=-1):
    if linear_sign not in (-1,1) or angular_sign not in (-1,1):raise ValueError('Signes +/-1 requis')
    if not math.isfinite(v) or not math.isfinite(w):raise ValueError('Commande non finie')
    v=linear_sign*v;w=angular_sign*w
    return f'M{v if abs(v)>=.00005 else 0.:.4f}:{w if abs(w)>=.00005 else 0.:.4f}\n'.encode('ascii')

class ArduinoLink:
    def __init__(self,port,log,linear_sign=-1,angular_sign=-1,*,factory=None,reset_delay=2.,timeout=.3):
        if factory is None:
            import serial
            factory=serial.Serial
        self.signs=(linear_sign,angular_sign);encode_command(0,0,*self.signs)
        self.serial=factory(port=port,baudrate=57600,timeout=.05,write_timeout=.1,exclusive=True)
        self.log=log;self.lock=threading.Lock();self.write_lock=threading.RLock();self.stop=threading.Event();self.failure=None;self.timeout=timeout;self.last=time.monotonic();self.stale=False
        try:
            self._write(encode_command(0,0,*self.signs),'initial_stop')
            time.sleep(reset_delay);self.send(0,0)
        except BaseException:
            self.serial.close();raise
        self.worker=threading.Thread(target=self.watchdog,daemon=True);self.reader=threading.Thread(target=self.receive,daemon=True)
        self.worker.start();self.reader.start()
    def _write(self,data,kind):
        with self.write_lock:
            count=self.serial.write(data)
            if count!=len(data):raise IOError('Ecriture serie incomplete')
            if kind=='command':self.last=time.monotonic();self.stale=False
        with self.lock:self.log.write(json.dumps({'monotonic_s':time.monotonic(),'direction':'TX','kind':kind,'line':data.decode().rstrip()})+'\n');self.log.flush()
    def send(self,v,w):
        if self.failure:raise RuntimeError(self.failure)
        self._write(encode_command(v,w,*self.signs),'command')
    def watchdog(self):
        while not self.stop.wait(.05):
            try:
                with self.write_lock:
                    if time.monotonic()-self.last>self.timeout:
                        self._write(encode_command(0,0,*self.signs),'stale_stop');self.stale=True
            except Exception as exc:self.failure=str(exc);return
    def receive(self):
        while not self.stop.is_set():
            try:
                data=self.serial.readline()
                if data:
                    with self.lock:self.log.write(json.dumps({'monotonic_s':time.monotonic(),'direction':'RX','line':data.decode('utf8',errors='replace').rstrip()})+'\n');self.log.flush()
            except Exception as exc:
                if not self.stop.is_set():self.failure=str(exc)
                return
    def close(self):
        self.stop.set()
        self.worker.join(timeout=1);self.reader.join(timeout=1)
        try:
            for _ in range(3):self._write(encode_command(0,0,*self.signs),'closing_stop');time.sleep(.03)
        finally:self.serial.close()
