#!/usr/bin/env python3
"""Builds the live preview of the PCB's web page (include/web_ui.h): the page with a
simulated controller behind it, to click through in any browser without a board. It is
preview/index.html, published to GitHub Pages by .github/workflows/pages.yml.

    python tools/web_ui_preview.py            # After editing the page or its settings
    python tools/web_ui_preview.py --check    # CI: fails if preview/index.html is stale
    python tools/web_ui_preview.py out.html   # Somewhere else

Phones first, as the page is: on a phone it fills the screen; on a wider screen it is shown
in a phone-sized frame.

The page is the firmware's own, byte for byte. The settings (names, labels, units, limits,
defaults), the ECU list and the MAP sensor presets are read from the same headers the
firmware is built from (standalone_settings.h, meth_tune.h, ecu_profiles.h,
engine_source.h), so the preview never drifts from the board. Only the data is made up: a
script answers the page's /api calls with a simulated engine (boost pulls every half
minute), pump, valve, line pressure, flow, trouble codes and profiles.
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INC = ROOT / 'include'


def read(name):
    return (INC / name).read_text(encoding='utf-8')


def page():
    src = read('web_ui.h')
    return src.split('R"HTML(', 1)[1].split(')HTML"', 1)[0]


def constants():
    """The few named constants the settings tables use, as numbers."""
    tune = read('meth_tune.h')
    eng = read('engine_source.h')
    profiles = ecus()
    sensors = re.search(r'kMapSensors\[\] = \{(.*?)\};', eng, re.S).group(1)
    return {
        'kShortestPulseMs': int(re.search(r'kShortestPulseMs = (\d+)', tune).group(1)),
        'kMeasuredNozzleMlMin': int(re.search(r'kMeasuredNozzleMlMin = (\d+)', tune).group(1)),
        'engine::kProfileCount': len(profiles),
        'engine::kMapSensorCount': len(re.findall(r'\{"', sensors)),
        'kSourceObd11': 2,
    }


def number(expr, consts):
    expr = expr.strip()
    expr = re.sub(r'static_cast<\w+>\((.*)\)', r'\1', expr)
    for name, value in sorted(consts.items(), key=lambda kv: -len(kv[0])):
        expr = expr.replace(name, str(value))
    if not re.fullmatch(r'[0-9A-Fa-fx\s+\-*/()]+', expr):
        raise ValueError('cannot read ' + expr)
    return int(eval(expr, {'__builtins__': {}}))  # Digits and arithmetic only (checked above)


def ecus():
    table = re.search(r'kProfiles\[\] = \{(.*?)\n\};', read('ecu_profiles.h'), re.S).group(1)
    out = []
    for m in re.finditer(r'\{"(\w+)", "([^"]+)", (\d+)UL, \w+, \w+, (0x[0-9A-Fa-f]+)UL, "(\w+)"', table):
        out.append({'label': m.group(2), 'status': m.group(5), 'kbit': int(m.group(3)) // 1000, 'base': int(m.group(4), 16)})
    return out


def map_sensors():
    sensors = re.search(r'kMapSensors\[\] = \{(.*?)\};', read('engine_source.h'), re.S).group(1)
    return re.findall(r'\{"([^"]+)"', sensors)


def settings():
    consts = constants()
    st = read('standalone_settings.h')
    tune_h = read('meth_tune.h')
    ranges = {}
    body = tune_h.split('inline Range rangeOf', 1)[1].split('default:', 1)[0]
    for m in re.finditer(r'case (k\w+): return \{([^,]+), ([^,]+), ([^}]+)\};', body):
        ranges[m.group(1)] = [number(x, consts) for x in m.groups()[1:]]

    def table(fn):
        part = st.split('inline Info ' + fn, 1)[1].split('default:', 1)[0]
        rows = []
        for m in re.finditer(r'case (?:meth_tune::)?(k\w+): return \{"([^"]*)", "([^"]*)", "([^"]*)", ([^,]+), ([^,]+), ([^,]+), (\d+)\}', part):
            key, name, label, unit, lo, hi, de, div = m.groups()
            if lo.strip() == 'r.min':
                lo, hi, de = ranges[key]
            else:
                lo, hi, de = number(lo, consts), number(hi, consts), number(de, consts)
            rows.append({'name': name, 'label': label, 'unit': unit, 'min': lo, 'max': hi, 'def': de, 'div': int(div)})
        return rows

    return table('tuneInfoOf'), table('infoOf')


SIMULATION = r'''
<script>
// ---- Preview only: a simulated controller answering the page's /api calls. ----
(function(){
const SPEC=__SPEC__;
const strip=document.createElement('div');strip.id='preview-strip';
strip.innerHTML='PREVIEW · simulated data · <a href="https://github.com/averyizatt/DIYWaterMethInjection/blob/main/docs/PCB.md">About the board</a>';
document.addEventListener('DOMContentLoaded',()=>document.body.appendChild(strip));
const all=SPEC.tune.concat(SPEC.own),info={};all.forEach(s=>info[s.name]=s);
const raw={};all.forEach(s=>raw[s.name]=s.def);
// A typical setup: a 250 psi transducer, the nozzle measured at 110 psi, a 2 gallon tank.
Object.assign(raw,{line_sensor_psi:250,line_min_psi:40,line_max_psi:220,nozzle_rated_psi:110,tank_l:76,meth_pct:50,start_psi:40,full_psi:180,armed:1});
let saved=JSON.parse(JSON.stringify(raw));
const shown=n=>raw[n]/info[n].div;
const sim={t0:Date.now(),last:Date.now(),line:0,used:2140,drive:0,amps:0,avg:null,peak:0,test:0,testPsi:null,testSum:0,testN:0,fault:'',
 wasOn:false,valveAt:0,offAt:0,budget:0,codes:[
 {id:'S07',text:'12 V supply low',action:'Injection held until it recovers',severity:1,active:0,count:2,first:[39,2],last:[42,1],freeze:{boost_psi:-9.8,rpm:180,amps:0,line_psi:0,supply_v:9.7,duty:0,air_c:12}},
 {id:'P06',text:'Line pressure too high (blocked nozzle or filter)',action:'Pump off, latched',severity:2,active:0,count:1,first:[40,811],last:[40,811],freeze:{boost_psi:15.2,rpm:5600,amps:8.9,line_psi:231,supply_v:13.8,duty:100,air_c:52}},
 {id:'W16',text:'Control pass late (over 100 ms)',action:'Logged',severity:0,active:0,count:3,first:[42,20],last:[42,400],freeze:{boost_psi:-10,rpm:850,amps:0,line_psi:60,supply_v:14.1,duty:0,air_c:30}}],
 events:5,profiles:[{name:'Street',used:1,values:null},{name:'Track',used:0},{name:'Race',used:0},{name:'Winter',used:0}]};
sim.profiles[0].values=JSON.parse(JSON.stringify(raw));
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
function engine(t){// A pull to full boost every 30 s, idle and cruise between.
 const p=t%30,baro=85.6;let map,rpm;
 if(p<14){map=36+Math.sin(t/3)*2;rpm=850+Math.sin(t)*15}
 else if(p<17){const k=(p-14)/3;map=40+k*(baro+shown('full_psi')/0.145*1.05-40);rpm=2600+k*2400}
 else if(p<25){const k=(p-17)/8;map=baro+(shown('full_psi')*1.05+Math.sin(t*4)*0.4)/0.145;rpm=5000+k*1600}
 else{map=30;rpm=Math.max(900,6600-(p-25)*1500)}
 return{map,rpm:Math.round(rpm),baro,boost:(map-baro)*0.1450377}}
function step(){const now=Date.now(),dt=Math.min(1,(now-sim.last)/1000),t=(now-sim.t0)/1000;sim.last=now;
 const e=engine(t),start=shown('start_psi'),full=shown('full_psi'),pwm=raw.pump_drive==1;
 let duty=0,on=0,hold='not armed';
 if(sim.test>0){sim.test-=dt;duty=100}
 else if(sim.fault)hold='fault';
 else if(raw.armed){hold='waiting for boost';
  if(e.boost>=start&&e.rpm>=shown('min_rpm')){const f=clamp((e.boost-start)/Math.max(0.1,full-start),0,1);hold='spraying';
   if(pwm)duty=shown('pwm_min_pct')+f*(shown('pwm_max_pct')-shown('pwm_min_pct'));
   else{const cyc=shown('cycle_ms'),onMs=shown('on_start_ms')+f*(shown('on_full_ms')-shown('on_start_ms'));on=Math.round(onMs);duty=(now%cyc)<onMs?100:0}}
  else if(e.rpm<shown('min_rpm')&&e.boost>=start)hold='engine speed below the minimum'}
 const pumpOn=duty>0;if(pumpOn&&!sim.wasOn){sim.valveAt=now+(now-sim.offAt>1000?shown('valve_lead_ms'):0)}if(!pumpOn&&sim.wasOn)sim.offAt=now;sim.wasOn=pumpOn;
 const valve=pumpOn&&now>=sim.valveAt;
 const target=pumpOn?(valve?112*Math.sqrt(duty/100):150):sim.line*0.97;sim.line+= (target-sim.line)*Math.min(1,dt*6);
 sim.amps=pumpOn?6.2*duty/100+Math.random()*0.25:0.02;if(pumpOn&&duty>=99.5){sim.avg=sim.avg===null?sim.amps:sim.avg+(sim.amps-sim.avg)*0.05;sim.peak=Math.max(sim.peak,sim.amps)}
 const rated=raw.nozzle_rated_psi,nz=shown('nozzle_ml_min');const flow=valve?(rated>0?nz*Math.sqrt(Math.max(0,sim.line)/rated):nz):0;
 sim.used+=flow*dt/60;sim.drive+=flow*dt/60;sim.budget=pumpOn?sim.budget+dt:Math.max(0,sim.budget-dt*shown('max_spray_s')/Math.max(1,shown('rest_s')));
 if(sim.test>0&&duty>=99.5){sim.testSum+=sim.line;sim.testN++;sim.testPsi=sim.testSum/sim.testN}
 const tank=raw.tank_l*100,state=sim.test>0?'pump test':sim.fault?'fault':pumpOn||hold==='spraying'?'spraying':raw.armed?'armed':'not armed';
 return{state,hold:sim.test>0?'pump test':hold,fault:sim.fault,armed:raw.armed,duty,boost_psi:e.boost,map_kpa:e.map,map_ok:1,baro_kpa:e.baro,baro_learned:1,rpm:e.rpm,rpm_ok:1,
  iat1:44+e.boost,iat2:30,pre_c:44+e.boost,ecu_iat:null,line_psi:raw.map_from==1||!raw.line_sensor_psi?null:Math.max(0,sim.line),amps:sim.amps,supply_v:13.9-sim.amps*0.04,rail_v:4.71,tank_low:0,
  can:{up:1,kbit:500,listen:0,bus_off:0,tx_err:0,rx_err:0,frames:Math.round((now-sim.t0)/50),age_ms:Math.round(Math.random()*40)},on_ms:on,cycle_ms:shown('cycle_ms'),
  unsaved:JSON.stringify(raw)!==JSON.stringify(saved)?1:0,uptime_s:Math.round(t)+1312,reset:'power on',crashes:0,trips:sim.fault?1:0,wifi_default:1,upload:'',
  flow:{ml_min:flow,from_line:rated>0&&raw.line_sensor_psi&&raw.map_from==0?1:0,used_ml:sim.used,drive_ml:sim.drive,tank_ml:tank,left_ml:tank>0?Math.max(0,tank-sim.used):-1,test_psi:sim.testPsi},
  valve_open:valve?1:0,amps_avg:sim.avg,amps_peak:sim.peak,
  wd:{late:3,worst_ms:141,keeper:0,task_ok:1,run_ms:Math.round(sim.budget*1000),run_limit_ms:raw.rest_s?shown('max_spray_s')*1000+8000:180000,heap:182000-Math.round(Math.random()*3000),heap_min:171000,stack_loop:5120,stack_can:2310,fault:''},
  codes:{faults:sim.codes.filter(c=>c.active&&c.severity==2).length,warnings:sim.codes.filter(c=>c.active&&c.severity==1).length,events:sim.events,any:sim.codes.length?1:0,power_ups:42},
  values:Object.fromEntries(all.map(s=>[s.name,shown(s.name)]))}}
const reply=(o,status=200)=>Promise.resolve(new Response(JSON.stringify(o),{status,headers:{'Content-Type':'application/json'}}));
const ok=(applied=0)=>reply({ok:1,applied}),no=p=>reply({ok:0,problem:p},400);
function setValue(name,value){const s=info[name];if(!s)return no('no such setting');let v=Math.round(parseFloat(value)*s.div);if(isNaN(v))return no('not a number');
 v=clamp(v,s.min,s.max);raw[name]=v;if(name==='map_sensor'&&v<SPEC.maps.length){raw.map_at_05v=[287,325,350][v]||raw.map_at_05v;raw.map_at_45v=[1881,2841,2350][v]||raw.map_at_45v}return ok(v)}
function action(a,s){const st=step();switch(a){
 case'arm':raw.armed=1;return ok();case'disarm':raw.armed=0;return ok();
 case'test':if(raw.armed)return no('disarm first');if(sim.fault)return no('clear the fault first');sim.test=clamp(+s||3,1,5);sim.testSum=0;sim.testN=0;sim.testPsi=null;return ok();
 case'stop':sim.test=0;return ok();case'clear':sim.fault='';return ok();case'clear_codes':sim.codes=[];sim.events++;return ok();
 case'refilled':sim.used=0;return ok();case'save':saved=JSON.parse(JSON.stringify(raw));return ok();
 case'revert':Object.assign(raw,saved);return ok();case'defaults':all.forEach(x=>{if(x.name!=='armed')raw[x.name]=x.def});return ok();
 default:return no('no such action')}}
const realFetch=window.fetch.bind(window);
window.fetch=async(url,opts={})=>{const u=new URL(url,location.href);const path=u.pathname;let body={};
 if(opts.body instanceof URLSearchParams)body=Object.fromEntries(opts.body);
 await new Promise(r=>setTimeout(r,40));
 if(path.endsWith('/api/meta'))return reply({tune:SPEC.tune,own:SPEC.own,ecus:SPEC.ecus,maps:SPEC.maps,ssid:'WaterMeth-3F2A',build:'PREVIEW0'});
 if(path.endsWith('/api/state'))return reply(step());
 if(path.endsWith('/api/codes'))return reply({power_up:42,uptime_s:1312,codes:sim.codes});
 if(path.endsWith('/api/profiles'))return reply(sim.profiles.map(p=>p.used?{name:p.name,used:1,start_psi:p.values.start_psi/10,full_psi:p.values.full_psi/10,meth_pct:p.values.meth_pct,drive:p.values.pump_drive}:{name:p.name,used:0}));
 if(path.endsWith('/api/profile')){const p=sim.profiles[+body.slot];if(!p)return no('no such profile');
  if(body.op==='save'){p.name=(body.name||p.name).slice(0,20);p.used=1;p.values=JSON.parse(JSON.stringify(raw));return ok()}
  if(!p.used)return no('that profile is empty or damaged');for(const k of ['pump_drive','line_min_psi','line_max_psi','nozzle_rated_psi','valve_lead_ms'].concat(SPEC.tune.map(x=>x.name)))raw[k]=p.values[k];saved=JSON.parse(JSON.stringify(raw));return ok()}
 if(path.endsWith('/api/set'))return setValue(body.name,body.value);
 if(path.endsWith('/api/action'))return action(body.a,body.s);
 if(path.endsWith('/api/wifi'))return (body.password||'').length<8?no('8 to 63 characters'):ok();
 if(path.endsWith('/update'))return no('this is a preview: nothing to install');
 return realFetch(url,opts)};
})();
</script>
'''


# Preview only. A phone-sized frame on wide screens, so it looks as it does on a phone (the
# page's own wide layout, four gauges a row, is held back inside the frame).
FRAME = '''<style>
#preview-strip{position:sticky;bottom:0;z-index:20;text-align:center;font:600 12px/1.4 system-ui;padding:7px 10px calc(7px + env(safe-area-inset-bottom));background:#3fa9ff;color:#04121f}
#preview-strip a{color:#04121f}
@media (min-width:700px){
 html{background:radial-gradient(circle at 50% 0,#1a2430,#05070a 70%);min-height:100%}
 body{max-width:420px;margin:28px auto;border:10px solid #1d242c;border-radius:42px;overflow:hidden;box-shadow:0 30px 80px #000c,0 0 0 1px #33404c;min-height:calc(100vh - 56px);display:flex;flex-direction:column}
 main{flex:1;width:100%}
 .gauges,.tiles{grid-template-columns:repeat(2,minmax(0,1fr))!important}.slots{grid-template-columns:1fr!important}.btns button{flex:1 1 auto!important}
}
</style>
'''


def build():
    tune, own = settings()
    spec = {'tune': tune, 'own': own, 'ecus': ecus(), 'maps': map_sensors()}
    html = page()
    sim = SIMULATION.replace('__SPEC__', json.dumps(spec))
    # Before the page's own script, so its first fetch already goes to the simulation.
    html = html.replace('<script>', sim + '<script>', 1)
    html = html.replace('<title>Water/meth controller</title>', '<title>Water/meth controller (preview)</title>')
    html = html.replace('</head>', FRAME + '</head>', 1)
    return html, f'{len(tune)} tuning and {len(own)} board settings, {len(spec["ecus"])} ECUs'


def main():
    args = sys.argv[1:]
    check = '--check' in args
    args = [a for a in args if a != '--check']
    out = Path(args[0]) if args else ROOT / 'preview' / 'index.html'
    html, what = build()
    if check:
        if not out.exists() or out.read_text(encoding='utf-8') != html:
            print(f'{out} is out of date: run python tools/web_ui_preview.py')
            sys.exit(1)
        print(f'{out} is up to date')
        return
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding='utf-8', newline='\n')
    print(f'{out}: {len(html)} bytes, {what}')


if __name__ == '__main__':
    main()
