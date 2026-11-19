"""Execute the production control/protocol C code with simulated sensor inputs.
Run: python Tests/test_control.py [path-to-clang-and-lld]
These tests do not simulate the STM32 electrical peripherals.
"""
from pathlib import Path
import ctypes as C
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
tools = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('F:/Program Files/MATLAB/R2023a/toolbox/sldrt/clang/win64')
out = root / 'Tests/build'
out.mkdir(exist_ok=True)
objects = []
for source in ['App/car_control.c', 'App/car_protocol.c', 'App/car_ps2.c', 'Tests/host_stubs.c']:
    obj = out / (Path(source).stem + '.obj')
    subprocess.run([str(tools/'clang.exe'), '--target=x86_64-pc-windows-msvc',
                    '-ffreestanding', '-fno-builtin', '-O1', '-IApp', '-ITests/host_include',
                    '-c', source, '-o', str(obj)], cwd=root, check=True)
    objects.append(str(obj))
exports = ['Control_Init', 'Control_SetMode', 'Control_Run', 'Control_Move',
           'Control_Speed', 'Control_IR', 'Control_Step', 'Control_State', 'Protocol_Byte',
           'Protocol_Poll', 'Protocol_Reset', 'Test_Servo', 'Test_Beep', 'PS2_Decode']
dll = out / 'car_tests.dll'
subprocess.run([str(tools/'lld.exe'), '-flavor', 'link', '/dll', '/noentry',
                '/nodefaultlib', '/out:'+str(dll), *objects,
                *['/export:'+n for n in exports]], check=True)
lib = C.CDLL(str(dll))
class Input(C.Structure):
    _fields_ = [(n, C.c_uint32) for n in ['now','line','obstacle_left','obstacle_right',
                'light_left','light_right','distance_mm','distance_valid','battery_mv','battery_valid','clock_ok','ps2_valid','ps2_buttons','ps2_lx','ps2_ly','sonar_valid','servo_angle']]
class Output(C.Structure):
    _fields_ = [('left',C.c_int32),('right',C.c_int32),('rgb',C.c_uint32),('beep',C.c_uint32),('servo',C.c_uint32)]
class State(C.Structure):
    _fields_ = [(n,C.c_uint32) for n in ['mode','running','speed','command','last_command',
                'phase','phase_since','turn_right','line_seen','line_last']] + [
                ('line_direction',C.c_int32),('battery_alarm',C.c_uint32),('fault',C.c_uint32),('lights_enabled',C.c_uint32)]
lib.Control_State.restype = C.POINTER(State)
lib.Control_Step.argtypes = [C.POINTER(Input), C.POINTER(Output)]
checks = []
def state(): return lib.Control_State().contents
def reset(mode=0):
    lib.Control_Init(); lib.Protocol_Reset(); lib.Control_SetMode(mode,0)
def step(now=10, **kw):
    i = Input(now, 6, 0, 0, 0, 0, 1000, 1, 7800, 1, 1, 0, 0, 128, 128, 1, 90)
    for key,value in kw.items(): setattr(i,key,value)
    o = Output(); lib.Control_Step(C.byref(i),C.byref(o)); return o
def stopped(o): return o.left == o.right == 0
def feed(text, now=0, port=0):
    for b in text.encode('ascii'): lib.Protocol_Byte(port,b,now)
def check(name, condition):
    assert condition, name
    checks.append(name)

reset(); check('boot is stopped', stopped(step()))
check('default speed is 3000',state().speed==3000)
for cmd,signs in [(1,(1,1)),(2,(-1,-1)),(3,(-1,1)),(4,(1,-1)),(5,(0,1)),(6,(1,0))]:
    lib.Control_Move(cmd,100); o=step(110)
    sign=lambda x:(x>0)-(x<0)
    check('manual direction '+str(cmd),(sign(o.left),sign(o.right)) == signs)
check('manual motion persists after 800ms',not stopped(step(901)) and state().running)
check('manual motion persists after 60 seconds',not stopped(step(60100)) and state().running)
lib.Control_Move(1,0xfffffff0);check('manual motion persists across tick wrap',step(900).left>0)
for stop_command in ['X','0','@STOP\n']:
    reset();feed('W',0);step(60000);feed(stop_command,60001)
    check('persistent motion stops with '+repr(stop_command),stopped(step(60002)) and not state().running)
lib.Control_Move(1,1);lib.Control_SetMode(1,2);check('mode change stops',stopped(step(3)))
reset(1);lib.Control_Run(1,0)
check('center black line goes straight',step(10).left==step(10).right>0)
o=step(20,line=1);check('left line turns left',o.left<o.right)
o=step(30,line=8);check('right line turns right',o.left>o.right)
check('lost line recovery is bounded',stopped(step(500,line=0)))
reset(1);lib.Control_Run(1,0);check('no line on start stays stopped',stopped(step(10,line=0)))
for mode in [2,4]:
    reset(mode);lib.Control_Run(1,0)
    check('sonar mode %d rejects no echo'%mode,stopped(step(10,distance_valid=0)))
    o=step(20,distance_mm=100)
    check('mode %d stops and looks left'%mode,stopped(o) and o.servo==150)
    o=step(100,distance_valid=0,sonar_valid=0,servo_angle=150)
    check('mode %d waits for settled left sample'%mode,stopped(o) and o.servo==150)
    o=step(400,distance_mm=600,distance_valid=0,servo_angle=150)
    check('mode %d looks right after left sample'%mode,stopped(o) and o.servo==30)
    o=step(800,distance_mm=1000,distance_valid=0,servo_angle=30)
    check('mode %d recenters before turning'%mode,stopped(o) and o.servo==90)
    check('mode %d waits for center echo'%mode,stopped(step(900,distance_valid=0,sonar_valid=0)))
    o=step(1200,distance_mm=100)
    check('mode %d chooses clearer right side'%mode,o.left>0 and o.right<0 and o.servo==90)
    check('mode %d completes bounded turn'%mode,stopped(step(1550)))
    check('mode %d drives after front clears'%mode,step(1560).left>0)
reset(2);lib.Control_Run(1,0);step(10,distance_mm=100)
step(410,distance_mm=1000,distance_valid=0,servo_angle=150)
step(810,distance_mm=400,distance_valid=0,servo_angle=30)
o=step(1210,distance_mm=100)
check('scanner chooses clearer left side',o.left<0 and o.right>0)
reset(2);lib.Control_Run(1,0);step(10,distance_mm=100)
step(910,distance_valid=0,sonar_valid=0,servo_angle=150)
o=step(1810,distance_valid=0,sonar_valid=0,servo_angle=30)
check('scanner both missing echoes stops without blind turn',stopped(o) and not state().running and o.servo==90)
reset(2);lib.Control_Run(1,0);step(10,distance_mm=100);feed('X',20)
o=step(25,distance_valid=0,servo_angle=150)
check('stop during scan recenters and parks',stopped(o) and o.servo==90 and not state().running)
reset(4);lib.Control_Run(1,0);step(10,distance_mm=100)
step(410,distance_mm=1000,distance_valid=0,servo_angle=150)
step(810,distance_mm=600,distance_valid=0,servo_angle=30,obstacle_left=1)
o=step(1210,distance_mm=100,obstacle_left=1)
check('combined scan excludes IR blocked side',o.left>0 and o.right<0)
check('combined turn stops for new side obstacle',stopped(step(1220,obstacle_right=1)) and not state().running)
reset(2);lib.Control_Run(1,0);step(10,distance_mm=100)
step(410,distance_mm=600,distance_valid=0,servo_angle=150)
step(810,distance_mm=1000,distance_valid=0,servo_angle=30)
check('center echo timeout disarms scan',stopped(step(1710,distance_valid=0,sonar_valid=0)) and not state().running)
reset(3);lib.Control_Run(1,0);step(10,obstacle_left=1)
o=step(270,obstacle_left=1);check('left obstacle turns right',o.left>0 and o.right<0)
reset(3);lib.Control_Run(1,0);step(10,obstacle_right=1)
o=step(270,obstacle_right=1);check('right obstacle turns left',o.left<0 and o.right>0)
reset(5);lib.Control_Run(1,0)
check('follow far target approaches',step(10,distance_mm=500).left>0)
check('follow near target retreats',step(20,distance_mm=100).left<0)
check('follow deadband stops',stopped(step(30,distance_mm=230)))
check('follow lost target stops',stopped(step(40,distance_mm=2000)))
reset(6);lib.Control_Run(1,0);o=step(10,light_left=1);check('left light steers left',o.left<o.right)
reset(7);check('RGB sequence',[step(t).rgb for t in range(0,3000,500)]==[1,0,2,0,4,0])
check('RGB mode never drives',all(stopped(step(t)) for t in range(0,3000,500)))
reset(8);lib.Control_Run(1,0);check('distance mode stays stopped',stopped(step()))
reset();lib.Control_Move(1,0)
o=step(100,battery_mv=3952)
check('low voltage alarms but permits motion',o.left>0 and o.right>0 and o.beep and state().battery_alarm and state().running)
o=step(260,battery_mv=0)
check('alarm pauses between beeps without motor lock',o.left>0 and not o.beep and state().battery_alarm)
check('battery hysteresis retains warning',step(1000,battery_mv=6900).beep and state().running)
o=step(1100,battery_mv=7000)
check('voltage recovery clears alarm and preserves motion',o.left>0 and not o.beep and not state().battery_alarm and state().running)
reset(7);check('low battery beeps in RGB mode',step(100,battery_mv=6500).beep)
reset();check('low battery beeps when parked',step(100,battery_mv=6500).beep and not state().running)
lib.Control_Move(1,40);check('invalid ADC stops',stopped(step(50,battery_valid=0)))
lib.Control_Move(1,60);check('wrong system clock stops',stopped(step(70,clock_ok=0)))
reset();feed('@SPEED=7000\n');check('speed frame',state().speed==7000)
feed('@SPEED=10001\n');check('out of range speed rejected',state().speed==7000)
feed('@MODE=4\n@RUN\n');check('select and start mode',state().mode==4 and state().running)
feed('@STOP\n');check('stop frame',not state().running)
feed('1',100);check('legacy ASCII forward',step(110).left>0)
lib.Protocol_Byte(1,2,120);check('legacy raw backward',step(130).left<0)
feed('@SERVO=160\n');check('servo command parks car',lib.Test_Servo()==160 and not state().running)
feed('@SERVO=0\n');check('servo bounds',lib.Test_Servo()==160)
feed('@RUN\n');check('run recenters servo',lib.Test_Servo()==90)
feed('@BEEP\n');check('buzzer command',lib.Test_Beep()==150)
feed('@'+'A'*60+'11111\n');check('overflow suffix cannot become movement',not state().running)
feed('@SPEED=',100);lib.Protocol_Poll(1200);feed('1111\n',1300)
check('timed out frame suffix discarded',not state().running and state().speed==7000)
feed('@MODE=',0,0);feed('@SPEED=4000\n',0,1);feed('1\n',0,0)
check('two ports have independent parsers',state().mode==1 and state().speed==4000)
feed('@MODE=999999999999999999999999999999\n');check('numeric overflow rejected',state().mode==1)
feed('1');feed('X');check('immediate stop command',not state().running)
# Decode actual protocol bytes, then exercise controller ownership and release.
class Pad(C.Structure):
    _fields_ = [(n,C.c_uint32) for n in ['buttons','lx','ly','id']]
lib.PS2_Decode.argtypes = [C.POINTER(C.c_uint8),C.c_uint,C.POINTER(Pad)]
def decode(data):
    pad=Pad(); raw=(C.c_uint8*len(data))(*data)
    return lib.PS2_Decode(raw,len(data),C.byref(pad)),pad
ok,pad=decode([255,0x41,0x5a,0xef,255])
check('PS2 digital up decodes with neutral axes',ok and pad.buttons==16 and pad.lx==pad.ly==128)
ok,pad=decode([255,0x73,0x5a,255,255,128,128,220,20])
check('PS2 analog left stick offsets',ok and pad.lx==220 and pad.ly==20)
for name,packet in [('absent',[255]*9),('low wire',[0]*9),('bad header',[255,0x73,0,255,255,128,128,128,128]),('truncated',[255,0x73,0x5a,255,255]),('config mode',[255,0xf3,0x5a,255,255,128,128,128,128])]:
    check('PS2 rejects '+name,not decode(packet)[0])
check('PS2 extended packet accepted',decode([255,0x79,0x5a,255,255]+[128]*16)[0])
reset();feed('P');check('P selects parked PS2 mode',state().mode==9 and not state().running)
feed('R');check('PS2 waits for stick neutral',stopped(step(ps2_valid=1,ps2_buttons=16)))
step(ps2_valid=1)
for button,signs in [(16,(1,1)),(64,(-1,-1)),(128,(-1,1)),(32,(1,-1))]:
    o=step(ps2_valid=1,ps2_buttons=button)
    check('PS2 dpad '+str(button),((o.left>0)-(o.left<0),(o.right>0)-(o.right<0))==signs)
check('PS2 release stops wheels',stopped(step(ps2_valid=1)) and state().running)
check('PS2 analog deadzone',stopped(step(ps2_valid=1,ps2_lx=140,ps2_ly=115)))
o=step(ps2_valid=1,ps2_lx=255,ps2_ly=0)
check('PS2 diagonal normalized to selected speed',0<=o.right<o.left<=3000)
o=step(ps2_valid=1,ps2_buttons=16,ps2_ly=255)
check('PS2 dpad overrides analog',o.left==o.right==3000)
check('PS2 invalid link stops and disarms',stopped(step(ps2_valid=0)) and not state().running)
check('PS2 reconnect does not restart',stopped(step(ps2_valid=1,ps2_buttons=16)))
for button in [1,16384]:
    feed('R');step(ps2_valid=1)
    check('PS2 stop button '+str(button),stopped(step(ps2_valid=1,ps2_buttons=button)) and not state().running)
feed('R');step(ps2_valid=1);feed('X')
check('serial stop holds despite PS2 direction',stopped(step(ps2_valid=1,ps2_buttons=16)))
feed('W');o=step(60000,ps2_valid=0,ps2_buttons=16384)
check('PS2 cannot override manual serial motion',state().mode==0 and o.left>0)
feed('@MODE=9\n');check('PS2 framed mode supported',state().mode==9 and not state().running)
for code,signs in [(64,(1,1)),(25,(-1,-1)),(7,(-1,1)),(9,(1,-1)),(22,(0,1)),(13,(1,0))]:
    reset();lib.Control_IR(code,0);o=step(60000)
    check('example24 infrared command '+str(code),((o.left>0)-(o.left<0),(o.right>0)-(o.right<0))==signs)
lib.Control_IR(28,60001)
check('infrared other key stops',stopped(step(60002)) and not state().running)
reset(2);lib.Control_Run(1,0);step(10,distance_mm=100)
feed('@SERVO=120\n',20)
o=step(25,servo_angle=120,distance_valid=0)
check('manual servo overrides scan recenter',stopped(o) and o.servo==120 and not state().running)
reset();feed('W');lib.Control_IR(21,100)
check('confirmed infrared 21 stops',stopped(step(110)) and not state().running)
reset();feed('W');feed('O');o=step()
check('O lights off preserves motion',not o.rgb and o.left>0 and state().running)
feed('H');check('H horn preserves motion',lib.Test_Beep()==150 and step().left>0)
feed('@BEEP\n');check('framed horn preserves motion',lib.Test_Beep()==150 and step().left>0)
feed('G');check('off persists through RGB mode',step(1000).rgb==0)
check('off preserves low battery beep',step(100,battery_mv=6500).beep and not step(100,battery_mv=6500).rgb)
feed('@LIGHT=1\n');check('light restore resumes RGB',step(1000).rgb==2 and state().lights_enabled)
feed('@LIGHT=0\n');check('framed lights off',step(1000).rgb==0 and not state().lights_enabled)
feed('@LIGHT=2\n');check('invalid light setting rejected',not state().lights_enabled)
reset();check('power on restores default lighting',state().lights_enabled and step(500).rgb==4)
reset();feed('W');lib.Control_IR(12,100)
o=step(110);check('infrared 12 horn preserves motion',o.beep and o.left>0 and state().running)
check('infrared horn ends at 150ms',not step(250).beep and state().running)
lib.Control_IR(94,300);o=step(310)
check('infrared 94 turns off RGB without stopping',not o.rgb and o.left>0 and not state().lights_enabled)
feed('@LIGHT=1\n',320,1);check('Bluetooth restores lights',step(330).rgb and state().lights_enabled)
feed('O',340,1);check('Bluetooth O turns off lights',not step(350).rgb and state().running)
feed('H',360,1);check('Bluetooth H requests horn',lib.Test_Beep()==150 and state().running)
lib.Control_IR(21,370);check('infrared 21 stops after light commands',stopped(step(380)) and not state().running)
reset();lib.Control_IR(12,0xfffffff0)
check('infrared horn handles tick wrap',step(20).beep and not step(200).beep)
report = '\n'.join('PASS '+n for n in checks)+'\n%d checks passed. Hardware not simulated.\n'%len(checks)
(root/'Tests/results.txt').write_text(report,encoding='utf-8')
print(report)
