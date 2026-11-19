#pragma once

// The stand-alone PCB build's web page, served from the board's own Wi-Fi access point
// (main_pcb.cpp, PCB_FROGDASH=0). Tabs:
//   Live         state, arm/disarm, gauges (boost, line pressure, flow, pump), test spray
//   Tuning       when and how much to spray (meth_tune.h, the pulse or PWM drive)
//   Nozzle       the nozzle (rating, or measured with a test spray at a known pressure),
//                the mix, and the fluid count against the tank
//   Hardware     the pressure input (line pressure transducer or MAP sensor), the solenoid
//                valve, the pump's current, the tank switch
//   Engine       where engine data comes from (OBD-II or an ECU's broadcast)
//   Diagnostics  trouble codes with freeze frames (diag.h), the watchdogs (watchdogs.h)
//   System       tune profiles, settings backup, Wi-Fi, firmware
// Every setting's name, label, unit and limits come from the board (/api/meta,
// standalone_settings.h); the page only groups them. A setting the page does not group
// still shows, under Tuning's "Other".
//
// Phones first: it is used from a phone in the car. Two gauges and two tiles a row from the
// narrowest phone (320 px) up, four of each on a wide screen; every control at least 44 px
// tall; text inputs at 16 px so iOS does not zoom in on them; the tabs scroll sideways.
// tools/web_ui_preview.py builds the page with a simulated controller for GitHub Pages.
namespace web_ui {

const char kPage[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#0c0f12"><title>Water/meth controller</title>
<style>
:root{color-scheme:dark;--bg:#0c0f12;--panel:#151a1f;--panel2:#1b2128;--line:#262e37;--text:#e9eef2;--dim:#8b98a5;--ok:#35d07f;--warn:#f2b63d;--bad:#f2564d;--acc:#3fa9ff;--acc2:#7cc4ff}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--text);font:15px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;-webkit-text-size-adjust:100%;-webkit-tap-highlight-color:transparent}
header{position:sticky;top:0;z-index:5;background:#0c0f12f2;border-bottom:1px solid var(--line);padding:10px 14px 0}
.top{display:flex;align-items:center;gap:10px}.logo{width:28px;height:28px;border-radius:8px;background:radial-gradient(circle at 30% 30%,var(--acc2),var(--acc) 60%,#145c99);box-shadow:0 0 12px #3fa9ff55}
.top h1{font-size:16px;margin:0;font-weight:650}.top .sub{color:var(--dim);font-size:12px}.pill{margin-left:auto;padding:4px 10px;border-radius:99px;font-size:12px;font-weight:700;background:var(--panel2);border:1px solid var(--line)}
nav{display:flex;gap:2px;overflow-x:auto;margin-top:8px;scrollbar-width:none}nav::-webkit-scrollbar{display:none}
nav button{background:none;border:0;border-bottom:2px solid transparent;color:var(--dim);padding:8px 12px;min-height:44px;font:inherit;font-size:15px;white-space:nowrap;cursor:pointer;border-radius:0}
nav button.on{color:var(--text);border-bottom-color:var(--acc)}nav .dot{display:inline-block;width:7px;height:7px;border-radius:50%;background:var(--bad);margin-left:5px;vertical-align:middle}
main{max-width:860px;margin:auto;padding:10px}.tab{display:none}.tab.on{display:block}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px;margin-bottom:12px}
.card h2{font-size:12px;margin:0 0 10px;color:var(--dim);font-weight:700;text-transform:uppercase;letter-spacing:.08em}
.card h3{font-size:13px;margin:14px 0 4px;color:var(--acc2);font-weight:650}
.hero{display:flex;align-items:center;gap:12px;flex-wrap:wrap}.hero .st{font-size:26px;font-weight:800;line-height:1.15}.hero .why{color:var(--dim)}
.hero .fault{color:var(--bad);font-weight:650;flex-basis:100%}.hero .fault:empty{display:none}.hero .grow{flex:1}
.arm{min-width:120px;min-height:56px;padding:12px 18px;font-size:17px;font-weight:800;border-radius:12px}
.gauges{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}
.g{background:var(--panel2);border:1px solid var(--line);border-radius:12px;padding:8px 8px 10px;text-align:center}
.g svg{width:100%;max-width:190px;display:block;margin:auto}.g .v{font-size:22px;font-weight:800;margin-top:-38px;font-variant-numeric:tabular-nums}.g .u{color:var(--dim);font-size:12px}.g .l{color:var(--dim);font-size:12px;margin-top:2px}
.tiles{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;margin-top:10px}
.t{background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:8px 10px}.t span{display:block;color:var(--dim);font-size:11px;text-transform:uppercase;letter-spacing:.05em}.t b{font-size:17px;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.row{display:grid;grid-template-columns:minmax(0,1fr) minmax(110px,40%);gap:10px;align-items:center;padding:9px 0;border-top:1px solid var(--line)}
.row:first-child{border-top:0}.row small{display:block;color:var(--dim);font-size:12px}
input,select{width:100%;min-height:44px;background:#0b0e11;color:var(--text);border:1px solid #33404c;border-radius:8px;padding:9px;font:inherit;font-size:16px}
input:focus,select:focus{outline:2px solid var(--acc);border-color:transparent}
.btns{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
button{background:#232b34;color:var(--text);border:1px solid #34404c;border-radius:10px;padding:10px 14px;min-height:44px;font:inherit;font-weight:600;cursor:pointer;touch-action:manipulation}
button:active{transform:translateY(1px)}button.p{background:var(--acc);border-color:var(--acc);color:#04121f}button.d{background:#3a1715;border-color:var(--bad);color:#ffd6d3}
button.ok{background:#0f3a24;border-color:var(--ok);color:#d5ffe8}button.sm{padding:6px 10px;font-size:13px}
.note{color:var(--dim);font-size:13px;margin:6px 0}.bad{color:var(--bad)}.ok{color:var(--ok)}.warn{color:var(--warn)}
.seg{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:6px}.seg button{padding:8px 4px;font-size:13px;line-height:1.2}.seg button.on{background:var(--acc);border-color:var(--acc);color:#04121f}
.bar{height:10px;background:#0b0e11;border-radius:6px;overflow:hidden;border:1px solid var(--line)}.bar i{display:block;height:100%;background:linear-gradient(90deg,var(--acc),var(--acc2))}
.code{border:1px solid var(--line);border-left:4px solid var(--dim);border-radius:10px;padding:10px;margin-bottom:8px;background:var(--panel2)}
.code.s2{border-left-color:var(--bad)}.code.s1{border-left-color:var(--warn)}.code .h{display:flex;gap:8px;align-items:baseline;flex-wrap:wrap}.code .id{font-weight:800;font-family:ui-monospace,monospace}
.code .now{font-size:11px;font-weight:800;color:#04121f;background:var(--warn);border-radius:5px;padding:1px 6px}.code .ff{color:var(--dim);font-size:12px;margin-top:4px}
.slots{display:grid;grid-template-columns:1fr;gap:8px}.slot{background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:10px}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:#000d;border:1px solid var(--line);padding:10px 16px;border-radius:10px;display:none;z-index:9}
.hide{display:none!important}
.btns button{flex:1 1 auto}
@media (min-width:700px){.gauges,.tiles{grid-template-columns:repeat(4,minmax(0,1fr))}.slots{grid-template-columns:repeat(2,1fr)}.btns button{flex:0 0 auto}}
</style></head><body>
<header><div class="top"><div class="logo"></div><div><h1>Water/meth controller</h1><div class="sub" id="ssid"></div></div><span class="pill" id="pill">...</span></div>
<nav id="nav"></nav></header>
<main>
<section class="tab" id="t-live">
 <div class="card"><div class="hero"><div class="grow"><div class="st" id="state">...</div><div class="why" id="hold"></div></div>
 <button class="arm p" id="arm">Arm</button><div class="fault" id="fault"></div></div></div>
 <div class="gauges" id="gauges"></div>
 <div class="card" style="margin-top:12px"><div class="tiles" id="tiles"></div>
 <div class="btns"><button id="test">Test spray 3 s</button><button id="stop">Stop</button><button id="clear">Clear faults</button><button class="p" id="save">Save settings</button></div>
 <p class="note" id="unsaved"></p></div>
</section>
<section class="tab" id="t-tune">
 <div class="card"><h2>Pump drive</h2><div id="g-drive"></div><p class="note"><b>Full-on pulses</b>: the pump runs at full pressure for part of each cycle, its on-time rising from the start-boost figure to the full-boost one: the best spray from an ordinary diaphragm pump. <b>PWM</b>: it runs continuously, its duty rising from the start-boost duty to the full-boost one, for pumps made for it.</p></div>
 <div class="card"><h2>When to spray</h2><div id="g-when"></div></div>
 <div class="card"><h2>How much</h2><div id="g-much"></div></div>
 <div class="card"><h2>Hot air</h2><div id="g-hot"></div></div>
 <div class="card"><h2>Limits</h2><div id="g-limits"></div></div>
 <div class="card" id="c-other"><h2>Other</h2><div id="g-other"></div></div>
</section>
<section class="tab" id="t-nozzle">
 <div class="card"><h2>Nozzle</h2><div id="g-nozzle"></div>
 <h3>From its rating</h3>
 <div class="row"><span>Nozzle rating<small>ml/min, as marked</small></span><select id="nzsize"><option>60</option><option>100</option><option>175</option><option>250</option><option>375</option><option>475</option><option>625</option><option>750</option><option>1000</option></select></div>
 <div class="row"><span>How many nozzles</span><select id="nzcount"><option>1</option><option>2</option><option>3</option><option>4</option></select></div>
 <div class="btns"><button id="nzuse">Use the rating</button></div>
 <h3>Better: measure it</h3>
 <p class="note">Disarmed, engine off: put the nozzle in a cup, run a <b>test spray</b>, weigh what it caught (water: 1 g is 1 ml). With a line pressure transducer the board also records the pressure during the test, so the flow can be worked out from the pressure while driving.</p>
 <div class="row"><span>Test length</span><select id="nzsecs"><option>3</option><option>5</option></select></div>
 <div class="btns"><button id="nztest">Run test spray</button></div>
 <div class="row"><span>Caught in the test<small>grams</small></span><input id="nzgrams" inputmode="decimal"></div>
 <p class="note" id="nzpsi"></p>
 <div class="btns"><button class="p" id="nzmeasured">Use the measurement</button></div></div>
 <div class="card"><h2>Mix</h2><div id="g-mix"></div><div class="seg" style="margin-top:8px"><button data-mix="0">Water only</button><button data-mix="30">30 % methanol</button><button data-mix="50">50 / 50</button></div>
 <p class="note">More methanol is more cooling and more fuel; above about 50 % it is a fire risk. In winter some methanol keeps it from freezing.</p></div>
 <div class="card"><h2>Fluid</h2><div class="tiles" id="fluid"></div><div class="bar" style="margin-top:10px"><i id="tankbar" style="width:0"></i></div><div id="g-tank" style="margin-top:6px"></div>
 <div class="btns"><button class="ok" id="refilled">Tank filled</button></div>
 <p class="note">The count adds up the flow while the valve is open: from line pressure when the nozzle was measured at a known pressure, otherwise from the nozzle figure. Kept through power cycles.</p></div>
</section>
<section class="tab" id="t-hw">
 <div class="card"><h2>Pressure input</h2>
 <div class="seg" id="pin"><button data-p="line">Line pressure transducer</button><button data-p="map">MAP sensor</button><button data-p="none">Nothing</button></div>
 <p class="note" id="pinnote"></p>
 <div id="p-line"><div class="row"><span>Transducer range<small>0.5-4.5 V, between the pump and the nozzle</small></span><select id="lrange"><option value="100">100 psi</option><option value="150">150 psi</option><option value="200">200 psi</option><option value="250">250 psi</option><option value="300">300 psi</option><option value="c">Other</option></select></div><div id="g-line"></div></div>
 <div id="p-map"><div id="g-map"></div></div></div>
 <div class="card"><h2>Solenoid valve</h2><div id="g-valve"></div><p class="note">A normally closed valve at the nozzle end of the line. It opens only while the pump runs and shuts the moment it stops, so nothing dribbles after a pulse, and a pump switch that fails shorted still cannot spray. With a lead time, after a stop the pump builds pressure against the shut valve first, so the first of the spray is fine mist, not a dribble. Between pulses the shut valve holds the pressure, so no lead is needed there.</p><div class="tiles" id="valvetiles"></div></div>
 <div class="card"><h2>Pump current</h2><div id="g-amps"></div><div class="tiles" id="amptiles"></div><p class="note">Measured on every pass (0.1 V per amp). The board stops the pump on a short (over 25 A), over the limit for 0.3 s, no current after 0.6 s (not connected), or current with the pump off (switch stuck on). The typical figure is this pump's own at full drive: a clogging filter or nozzle draws more, a worn pump less.</p></div>
 <div class="card"><h2>Tank switch</h2><div id="g-tankswitch"></div></div>
</section>
<section class="tab" id="t-engine">
 <div class="card"><h2>Engine data on CAN</h2><p class="note" id="source"></p><div id="g-engine"></div></div>
</section>
<section class="tab" id="t-diag">
 <div class="card"><h2>Trouble codes</h2><p class="note" id="codesnote"></p><div id="codes"></div>
 <div class="btns"><button id="clear2">Clear faults</button><button class="d" id="clearcodes">Clear codes</button></div>
 <p class="note">Codes are kept through power cycles with the readings at the moment each was last set. <b>Clear faults</b> lets the pump run again after a latched fault; <b>Clear codes</b> empties this list.</p></div>
 <div class="card"><h2>Watchdogs</h2><div class="tiles" id="wd"></div><p class="note">Each watches one thing on its own: the task watchdog on both cores; the pump keeper on the other core (pump and valve off if control stops setting them for 0.25 s); the outputs read back each pass; the pump's on-time kept to the longest spray (the spray budget); the CAN task's heartbeat; the settings checked every second; memory; and the crash hook that switches the outputs off before a restart.</p></div>
</section>
<section class="tab" id="t-sys">
 <div class="card"><h2>Tune profiles</h2><p class="note">Each profile keeps how the system sprays: all of the tuning, the pump drive, the nozzle's pressure, the valve lead and the line pressure limits. Not the wiring, engine data or arming. Loading one keeps it.</p><div class="slots" id="slots"></div></div>
 <div class="card"><h2>Backup</h2><p class="note">Every setting as a file, to keep or to put on another board.</p><div class="btns"><button id="export">Download settings</button></div><div class="row"><span>Restore from a file</span><input type="file" id="import" accept=".json"></div><p class="note" id="importnote"></p>
 <div class="btns"><button id="revert">Undo unsaved changes</button><button class="d" id="defaults">All defaults</button></div></div>
 <div class="card"><h2>Wi-Fi</h2><p class="note" id="wifinote"></p><div id="g-wifi"></div>
 <div class="row"><span>New password<small>8 to 63 characters, used from the next power-up</small></span><input id="pass" type="password" autocomplete="new-password"></div>
 <div class="btns"><button id="setpass">Set password</button></div></div>
 <div class="card"><h2>Firmware</h2><p class="note">Only the <b>WaterMethPCBStandalone</b> image: any other is checked and refused. Not while injecting. Running build <span id="build"></span>.</p>
 <input type="file" id="fw" accept=".bin"><div class="btns"><button id="upload">Install</button></div><p class="note" id="upnote"></p></div>
 <div class="card"><h2>Since start-up</h2><p class="note" id="since"></p></div>
</section>
</main><div id="toast"></div>
<script>
const $=id=>document.getElementById(id);let meta=null,last=null,info={},codesSeen=-1,tab='live';
const TABS=[['live','Live'],['tune','Tuning'],['nozzle','Nozzle & fluid'],['hw','Hardware'],['engine','Engine'],['diag','Diagnostics'],['sys','System']];
const GROUPS={drive:['pump_drive'],when:['start_psi','full_psi','min_rpm','early_rpm','min_air_c'],
 much:['cycle_ms','on_start_ms','on_full_ms','ramp_ms','pwm_min_pct','pwm_max_pct'],hot:['hot_from_c','hot_full_c'],
 limits:['max_spray_s','rest_s','max_dose_pct','overboost_assist','engine_l'],nozzle:['nozzle_ml_min','nozzle_rated_psi'],mix:['meth_pct'],tank:['tank_l'],
 line:['line_min_psi','line_max_psi'],map:['map_sensor','map_at_05v','map_at_45v','baro'],valve:['valve_lead_ms'],amps:['pump_amps_max'],
 tankswitch:['tank_switch_closed_when_low','tank_protection'],engine:['source','ecu','base_id','bitrate','listen_only'],wifi:['wifi_always']};
const HIDDEN=['armed','map_from','line_sensor_psi'];
const PULSE_ONLY=['cycle_ms','on_start_ms','on_full_ms','ramp_ms'],PWM_ONLY=['pwm_min_pct','pwm_max_pct'];
const SOURCES=['Nothing (a MAP sensor only)','An ECU\'s CAN broadcast','OBD-II (most cars)','OBD-II, 29-bit IDs'];
function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(e._t);e._t=setTimeout(()=>e.style.display='none',2600)}
async function post(url,data){const r=await fetch(url,{method:'POST',body:new URLSearchParams(data)}).catch(()=>null);const j=r?await r.json().catch(()=>({ok:0,problem:'no answer'})):{ok:0,problem:'no connection'};if(!j.ok)toast(j.problem||'refused');return j}
function choices(s){if(s.name==='source')return SOURCES;if(s.name==='pump_drive')return['Full-on pulses (most pumps)','PWM: duty rises with boost'];
 if(s.name==='ecu')return meta.ecus.map(e=>e.label+(e.status==='car'?'':' (from its document)'));if(s.name==='map_sensor')return[...meta.maps,'Your own two points'];
 if(s.max===1&&s.min===0)return['No','Yes'];return null}
function field(s){if(!s)return'';const c=choices(s);const lim=c?'':`${s.min/s.div}-${s.max/s.div}`;const help=`<small>${[s.unit,lim].filter(Boolean).join(' · ')}</small>`;
 const input=c?`<select data-name="${s.name}">${c.map((t,i)=>`<option value="${i}">${t}</option>`).join('')}</select>`:`<input data-name="${s.name}" inputmode="decimal" type="text">`;
 return `<div class="row" data-row="${s.name}"><span>${s.label}${help}</span>${input}</div>`}
const set=(name,value)=>post('/api/set',{name,value}).then(j=>{if(j.ok){toast((info[name]?info[name].label:name)+': '+(j.applied/(info[name]?info[name].div:1)));poll()}return j});
function build(){meta.tune.concat(meta.own).forEach(s=>info[s.name]=s);const used=new Set(HIDDEN);
 for(const [g,names] of Object.entries(GROUPS)){const el=$('g-'+g);if(!el)continue;el.innerHTML=names.map(n=>{used.add(n);return field(info[n])}).join('')}
 const other=Object.keys(info).filter(n=>!used.has(n));$('g-other').innerHTML=other.map(n=>field(info[n])).join('');$('c-other').classList.toggle('hide',!other.length);
 $('ssid').textContent=meta.ssid+' · 192.168.4.1';$('build').textContent=meta.build;
 document.querySelectorAll('[data-name]').forEach(e=>e.addEventListener('change',()=>set(e.dataset.name,e.value)));
 $('nav').innerHTML=TABS.map(([k,t])=>`<button data-tab="${k}">${t}${k==='diag'?'<span class="dot hide" id="diagdot"></span>':''}</button>`).join('');
 document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>show(b.dataset.tab));show(location.hash.slice(1)||'live')}
function show(k){if(!TABS.some(t=>t[0]===k))k='live';tab=k;history.replaceState(null,'','#'+k);document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('on',t.id==='t-'+k));
 document.querySelectorAll('[data-tab]').forEach(b=>b.classList.toggle('on',b.dataset.tab===k));if(k==='diag')loadCodes();if(k==='sys')loadSlots()}
const f=(v,d=1)=>v===null||v===undefined||isNaN(v)?'-':Number(v).toFixed(d);
function arc(frac,color){const a0=-210,a1=30,r=70,cx=90,cy=86;const pt=a=>[cx+r*Math.cos(a*Math.PI/180),cy+r*Math.sin(a*Math.PI/180)];
 const p=(a,b)=>{const[x0,y0]=pt(a),[x1,y1]=pt(b);return`M${x0} ${y0}A${r} ${r} 0 ${b-a>180?1:0} 1 ${x1} ${y1}`};frac=Math.max(0,Math.min(1,frac||0));
 return`<svg viewBox="0 0 180 120"><path d="${p(a0,a1)}" stroke="#262e37" stroke-width="12" fill="none" stroke-linecap="round"/>${frac>0.005?`<path d="${p(a0,a0+(a1-a0)*frac)}" stroke="${color}" stroke-width="12" fill="none" stroke-linecap="round"/>`:''}</svg>`}
function gauge(label,value,unit,frac,color,sub){return`<div class="g">${arc(frac,color)}<div class="v">${value}</div><div class="u">${unit}</div><div class="l">${label}${sub?' · '+sub:''}</div></div>`}
const tile=(l,v,c='')=>`<div class="t"><span>${l}</span><b class="${c}">${v}</b></div>`;
function render(s){last=s;const v=s.values,st=s.state;
 $('state').textContent=st[0].toUpperCase()+st.slice(1);$('state').className='st '+(st==='fault'?'bad':st==='spraying'||st==='pump test'||st==='armed'?'ok':'');
 $('pill').textContent=st.toUpperCase();$('pill').style.color=st==='fault'?'var(--bad)':st==='not armed'?'var(--dim)':'var(--ok)';
 $('hold').textContent=st==='armed'||st==='not armed'?s.hold:'';$('fault').textContent=s.fault;
 $('arm').textContent=s.armed?'Disarm':'Arm';$('arm').className='arm '+(s.armed?'d':'p');
 const full=v.full_psi||20,lineMax=v.line_sensor_psi||250,nz=v.nozzle_ml_min||1;
 $('gauges').innerHTML=gauge('Boost',s.map_ok?f(s.boost_psi):'-','psi',(s.boost_psi||0)/(full*1.25),s.boost_psi>=v.start_psi?'var(--ok)':'var(--acc)',s.map_ok?'start '+v.start_psi:'no reading')
  +(v.map_from==1||!v.line_sensor_psi?'':gauge('Line pressure',s.line_psi===null?'-':f(s.line_psi,0),'psi',(s.line_psi||0)/lineMax,'var(--acc2)',s.line_psi===null?'no reading':''))
  +gauge('Flow',f(s.flow.ml_min,0),'ml/min',s.flow.ml_min/(nz*1.2),'#9b7cff',s.flow.ml_min>0?(s.flow.from_line?'from pressure':'nozzle figure'):'')
  +gauge('Pump',f(s.duty,0)+'%',f(s.amps,1)+' A',s.duty/100,s.duty>0?'var(--ok)':'var(--acc)',s.valve_open?'valve open':'valve shut');
 const left=s.flow.left_ml;
 $('tiles').innerHTML=tile('Engine',s.rpm_ok?s.rpm+' rpm':'-',s.rpm_ok?'':'warn')+tile('Manifold',s.map_ok?f(s.map_kpa)+' kPa':'-',s.map_ok?'':'warn')
  +tile('Barometer',f(s.baro_kpa)+(s.baro_learned?'':' *'))+tile('Air',s.pre_c===null?(s.ecu_iat===null?'-':f(s.ecu_iat)+' °C ecu'):f(s.pre_c)+' °C')
  +tile('Supply',f(s.supply_v)+' V',s.supply_v<10.5||s.supply_v>16.5?'warn':'')+tile('Tank',s.tank_low?'LOW':left>=0?f(left/1000,2)+' L':'OK',s.tank_low?'bad':'ok')
  +(v.pump_drive==1?tile('PWM duty',f(s.duty,0)+' %'):tile('Pulse',s.on_ms?s.on_ms+' / '+s.cycle_ms+' ms':'-'))+tile('Used this drive',f(s.flow.drive_ml,0)+' ml');
 const pwm=v.pump_drive==1;document.querySelectorAll('[data-row]').forEach(r=>{const n=r.dataset.row;if(PULSE_ONLY.includes(n)||PWM_ONLY.includes(n))r.classList.toggle('hide',PWM_ONLY.includes(n)!==pwm)});
 $('unsaved').textContent=s.unsaved?'Changed settings are not saved yet: Save settings keeps them through power cycles.':'';
 const c=s.can;$('source').innerHTML=!c.up?'<span class="bad">CAN not running.</span>':c.bus_off?'<span class="bad">CAN bus-off: check CAN H / CAN L and the speed.</span>'
  :(v.source==0?'No engine data on CAN: a MAP sensor alone, so no engine-speed gate and no dose limit.':c.frames?`<span class="ok">Hearing the engine: ${c.frames} frames, last ${c.age_ms} ms ago.</span>`
  :'<span class="warn">Nothing heard from the engine yet. Check the wiring, the ECU and the CAN speed; for OBD-II, the ignition must be on.</span>')+` ${c.kbit} kbit/s${c.listen?', listening only':''}.`
  +(v.source>=2&&v.map_from==0&&!s.map_ok&&s.rpm_ok?' <span class="warn">The car answers but gives no manifold pressure: fit a MAP sensor and choose it under Hardware.</span>':'');
 const mode=v.map_from==1?'map':v.line_sensor_psi>0?'line':'none';document.querySelectorAll('#pin [data-p]').forEach(b=>b.classList.toggle('on',b.dataset.p===mode));
 $('p-line').classList.toggle('hide',mode!=='line');$('p-map').classList.toggle('hide',mode!=='map');
 if(document.activeElement!==$('lrange'))$('lrange').value=[100,150,200,250,300].includes(v.line_sensor_psi)?String(v.line_sensor_psi):'c';
 $('pinnote').innerHTML=mode==='line'?(s.line_psi===null?'<span class="warn">No reading from the transducer: check its wiring (5 V, signal, ground).</span>':`Reading ${f(s.line_psi,0)} psi. Pressure checks and the flow from pressure are on.`)
  :mode==='map'?'Manifold pressure from the sensor; engine speed still from CAN. No line pressure checks.':'No sensor on the pressure input: no line pressure checks, and flow is taken from the nozzle figure.';
 $('nzpsi').innerHTML=s.flow.test_psi!==null?`Line pressure in the last test: <b>${f(s.flow.test_psi,0)} psi</b>. It is stored with the measurement.`:(mode==='line'?'The last test\'s line pressure shows here once one has run.':'');
 $('fluid').innerHTML=tile('Flow now',f(s.flow.ml_min,0)+' ml/min')+tile('Used since filled',f(s.flow.used_ml/1000,2)+' L')+tile('Left',left>=0?f(left/1000,2)+' L':'set the tank size',left>=0&&left<s.flow.tank_ml*0.15?'warn':'')+tile('This drive',f(s.flow.drive_ml,0)+' ml');
 $('tankbar').style.width=(left>=0&&s.flow.tank_ml>0?Math.round(100*left/s.flow.tank_ml):0)+'%';
 $('valvetiles').innerHTML=tile('Valve',s.valve_open?'OPEN':'shut',s.valve_open?'ok':'')+tile('Pump',f(s.duty,0)+' %');
 $('amptiles').innerHTML=tile('Now',f(s.amps,2)+' A')+tile('Typical at full',s.amps_avg===null?'not yet':f(s.amps_avg,2)+' A')+tile('Peak',f(s.amps_peak,2)+' A');
 const w=s.wd;$('wd').innerHTML=tile('Watchdog fault',w.fault||'none',w.fault?'bad':'ok')+tile('CAN task',w.task_ok?'running':'STOPPED',w.task_ok?'ok':'bad')
  +tile('Late passes',w.late+' (worst '+w.worst_ms+' ms)')+tile('Keeper trips',w.keeper,w.keeper?'warn':'')+tile('Spray budget',f(w.run_ms/1000,1)+' / '+f(w.run_limit_ms/1000,0)+' s')
  +tile('Memory',Math.round(w.heap/1024)+' KB (min '+Math.round(w.heap_min/1024)+')',w.heap_min<24576?'warn':'')+tile('Stack left',w.stack_loop+' / '+w.stack_can+' B');
 const k=s.codes;$('diagdot').classList.toggle('hide',!(k.faults||k.warnings));
 $('codesnote').textContent=(k.faults||k.warnings)?`${k.faults} fault(s) and ${k.warnings} warning(s) present now.`:'Nothing present now.';
 if(tab==='diag'&&k.events!==codesSeen)loadCodes();
 $('wifinote').innerHTML=s.wifi_default?'<span class="warn">Still on the default password "watermeth": set your own.</span>':'Your own password is set.';
 $('since').textContent=`Up ${s.uptime_s} s (power-up ${k.power_ups}); last reset: ${s.reset}; crashes this ignition cycle: ${s.crashes}; pump faults: ${s.trips}.`;
 if(s.upload)$('upnote').textContent=s.upload;
 document.querySelectorAll('[data-name]').forEach(e=>{if(document.activeElement===e)return;const x=v[e.dataset.name];if(x!==undefined)e.value=String(x)})}
async function poll(){try{const r=await fetch('/api/state');render(await r.json())}catch(e){$('state').textContent='No connection';$('pill').textContent='OFFLINE'}}
async function loadCodes(){try{const j=await(await fetch('/api/codes')).json();codesSeen=last?last.codes.events:-1;
 $('codes').innerHTML=j.codes.length?j.codes.map(c=>`<div class="code s${c.severity}"><div class="h"><span class="id">${c.id}</span><span>${c.text}</span>${c.active?'<span class="now">NOW</span>':''}</div>
 <div class="note">${c.action}. Seen ${c.count}×; first at power-up ${c.first[0]} ${c.first[1]} s, last at power-up ${c.last[0]} ${c.last[1]} s.</div>
 <div class="ff">At the last: boost ${f(c.freeze.boost_psi)} psi · ${c.freeze.rpm} rpm · pump ${c.freeze.duty} % ${f(c.freeze.amps,2)} A · line ${f(c.freeze.line_psi,0)} psi · ${f(c.freeze.supply_v,2)} V · air ${c.freeze.air_c} °C</div></div>`).join('')
 :'<p class="note ok">No codes stored.</p>'}catch(e){}}
async function loadSlots(){try{const j=await(await fetch('/api/profiles')).json();
 $('slots').innerHTML=j.map((p,i)=>`<div class="slot"><input value="${p.name.replace(/"/g,'')}" maxlength="20" id="sn${i}"><div class="note">${p.used?`start ${p.start_psi} · full ${p.full_psi} psi · ${p.meth_pct} % meth · ${p.drive==1?'PWM':'pulses'}`:'empty'}</div>
 <div class="btns"><button class="sm" data-save="${i}">Save current</button>${p.used?`<button class="sm p" data-load="${i}">Load</button>`:''}</div></div>`).join('');
 document.querySelectorAll('[data-save]').forEach(b=>b.onclick=async()=>{const i=b.dataset.save;if((await post('/api/profile',{op:'save',slot:i,name:$('sn'+i).value})).ok){toast('Profile saved');loadSlots()}});
 document.querySelectorAll('[data-load]').forEach(b=>b.onclick=async()=>{const i=b.dataset.load;if((await post('/api/profile',{op:'load',slot:i})).ok){toast('Profile loaded and kept');poll()}})}catch(e){}}
$('arm').onclick=()=>post('/api/action',{a:last&&last.armed?'disarm':'arm'}).then(poll);
$('test').onclick=()=>post('/api/action',{a:'test',s:3}).then(poll);$('stop').onclick=()=>post('/api/action',{a:'stop'}).then(poll);
$('clear').onclick=$('clear2').onclick=()=>post('/api/action',{a:'clear'}).then(j=>{if(j.ok)toast('Faults cleared');poll()});
$('clearcodes').onclick=()=>{if(confirm('Clear every stored code?'))post('/api/action',{a:'clear_codes'}).then(()=>{loadCodes();poll()})};
$('save').onclick=()=>post('/api/action',{a:'save'}).then(j=>{if(j.ok)toast('Saved (written once away from boost)');poll()});
$('revert').onclick=()=>post('/api/action',{a:'revert'}).then(j=>{if(j.ok)toast('Back to the saved settings');poll()});
$('defaults').onclick=()=>{if(confirm('Every setting back to its default? (Save settings keeps it.)'))post('/api/action',{a:'defaults'}).then(poll)};
$('refilled').onclick=()=>post('/api/action',{a:'refilled'}).then(j=>{if(j.ok)toast('Fluid count reset');poll()});
$('nzuse').onclick=()=>set('nozzle_ml_min',Number($('nzsize').value)*Number($('nzcount').value));
$('nztest').onclick=()=>post('/api/action',{a:'test',s:$('nzsecs').value}).then(poll);
$('nzmeasured').onclick=async()=>{const g=parseFloat($('nzgrams').value);if(!(g>0)){toast('Enter the grams caught');return}
 await set('nozzle_ml_min',Math.round(g/Number($('nzsecs').value)*60));if(last&&last.flow.test_psi!==null)await set('nozzle_rated_psi',Math.round(last.flow.test_psi))};
document.querySelectorAll('[data-mix]').forEach(b=>b.onclick=()=>set('meth_pct',b.dataset.mix));
document.querySelectorAll('#pin [data-p]').forEach(b=>b.onclick=async()=>{const p=b.dataset.p;
 if(p==='map'){await set('map_from',1);return}await set('map_from',0);
 if(p==='none')await set('line_sensor_psi',0);else if(!last||!last.values.line_sensor_psi)await set('line_sensor_psi',250)});
$('lrange').onchange=()=>{const x=$('lrange').value;if(x==='c'){const y=prompt('Transducer range, psi at 4.5 V');if(y)set('line_sensor_psi',y)}else set('line_sensor_psi',x)};
$('setpass').onclick=()=>post('/api/wifi',{password:$('pass').value}).then(j=>{if(j.ok){toast('Password set');$('pass').value=''}});
$('export').onclick=()=>{if(!last)return;const v={...last.values};delete v.armed;const blob=new Blob([JSON.stringify({format:'watermeth-pcb-settings',version:1,values:v},null,1)],{type:'application/json'});
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='watermeth-settings.json';a.click()};
$('import').onchange=async e=>{const file=e.target.files[0];if(!file)return;let j;try{j=JSON.parse(await file.text())}catch(x){$('importnote').textContent='Not a settings file.';return}
 if(!j||j.format!=='watermeth-pcb-settings'||!j.values){$('importnote').textContent='Not a settings file from this board.';return}
 // A MAP sensor preset last: setting its two points by hand marks the sensor "your own".
 const entries=Object.entries(j.values).sort((a,b)=>(a[0]==='map_sensor')-(b[0]==='map_sensor'));
 let n=0,bad=0;for(const[k,x]of entries){if(k==='armed'||!info[k])continue;const r=await post('/api/set',{name:k,value:x});r.ok?n++:bad++;$('importnote').textContent=`Restoring... ${n}`}
 $('importnote').textContent=`Restored ${n} settings${bad?`, ${bad} refused`:''}. Save settings keeps them.`;poll()};
$('upload').onclick=async()=>{const file=$('fw').files[0];if(!file)return;$('upnote').textContent='Installing...';const body=new FormData();body.append('fw',file);
 const r=await fetch('/update',{method:'POST',body}).catch(()=>null);const j=r?await r.json().catch(()=>({})):{};
 $('upnote').textContent=j.ok?'Installed: the board restarts into it, and keeps it once it has run a minute.':'Refused: '+(j.problem||'no answer')};
fetch('/api/meta').then(r=>r.json()).then(m=>{meta=m;build();poll();setInterval(poll,700)});
</script></body></html>)HTML";

} // namespace web_ui
