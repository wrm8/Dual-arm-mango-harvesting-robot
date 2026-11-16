'use strict';
const $ = id => document.getElementById(id);
const colors = ['#66d9cf','#ffbd71','#91b7ff','#de9df4'];
const wheelNames = ['A 左后','B 左前','C 右前','D 右后'];
const labels = {collecting:'采集中',delayed:'起转有时间差',within_resolution:'阈值内未见明显差异',insufficient:'数据不足',interrupted:'采集中断',not_started:'有轮未起转'};
let latest = null, lastSuccess = 0;
function node(tag,text,cls){const el=document.createElement(tag);if(text!==undefined)el.textContent=text;if(cls)el.className=cls;return el;}
function n(v,d=3){return Number.isFinite(v)?v.toFixed(d):'—';}
function topic(s,name){return s.topics[name]||{message:null,age_s:null,hz:0};}
function age(t){return t.age_s===null?'未收到':`${n(t.age_s,2)} s 前 · ${n(t.hz,1)} Hz`;}
function stale(t,limit=.5){return t.age_s===null||t.age_s>limit;}
function draw(id,history,yfn,series,unit){
 const c=$(id),dpr=window.devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
 c.width=w*dpr;c.height=h*dpr;const ctx=c.getContext('2d');ctx.scale(dpr,dpr);
 const p={l:55,r:18,t:18,b:30},pw=w-p.l-p.r,ph=h-p.t-p.b;
 const items=history.filter(x=>history.length && x.t>=history[history.length-1].t-12);
 const values=items.flatMap(x=>yfn(x).filter(Number.isFinite));
 let lo=Math.min(0,...values),hi=Math.max(0,...values);if(hi-lo<.1){hi+=.05;lo-=.05;}const pad=(hi-lo)*.1;lo-=pad;hi+=pad;
 ctx.font='11px system-ui';ctx.fillStyle='#9aaec8';ctx.strokeStyle='#2a3a51';
 for(let i=0;i<5;i++){const y=p.t+i*ph/4;ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.fillText((hi-i*(hi-lo)/4).toFixed(unit==='°'?1:2),3,y+4);}
 ctx.fillText(unit,5,12);ctx.fillText('最近 12 秒',p.l,h-5);
 if(!items.length)return;const end=items[items.length-1].t,start=end-12;
 for(let k=0;k<series;k++){ctx.beginPath();ctx.strokeStyle=colors[k%4];ctx.setLineDash(k>=4?[5,4]:[]);let first=true,prev=null;for(const x of items){const v=yfn(x)[k];if(!Number.isFinite(v)){first=true;continue;}const px=p.l+(x.t-start)/12*pw,py=p.t+(hi-v)/(hi-lo)*ph;if(first||(prev!==null&&x.t-prev>.4))ctx.moveTo(px,py);else ctx.lineTo(px,py);first=false;prev=x.t;}ctx.stroke();}ctx.setLineDash([]);
}
function render(s){
 const demo=s.mode==='demo';$('banner').textContent=demo?'演示模式：以下是模拟数据，故意模拟 A 左后轮晚约 250 ms 起转；没有连接机器人。':'实时监测 · 仅订阅数据。请先打开本页面，再由你启动任务；页面不会启动底盘或机械臂。';
 $('faults').textContent=Object.entries(s.faults).map(([k,v])=>`${k}: ${v}`).join('\n');
 $('stage').textContent=s.command.stage??'—';$('commandSource').textContent=`${s.command.topic||'等待指令'} · ${s.command.age_s===null?'未收到':n(s.command.age_s,2)+' s 前'}`;
 const ws=topic(s,'/base/wheel_state'),bs=topic(s,'/base/state'),m=bs.message||{};
 $('yaw').textContent=`${n(Number.isFinite(m.yaw_rad)?m.yaw_rad*180/Math.PI:null,2)} °${stale(ws,s.settings.feedback_timeout_s)?'（过期）':''}`;$('vel').textContent=`vx ${n(m.vx_mps)} / vy ${n(m.vy_mps)} m/s · wz ${n(m.wz_rad_s)} rad/s`;
 $('feedback').textContent=stale(ws,s.settings.feedback_timeout_s)?'反馈过期 / 未收到':`${n(ws.hz,1)} Hz`;
 $('baseStatus').textContent=stale(bs)?'状态过期':m.command_timed_out?'指令超时':stale(ws,s.settings.feedback_timeout_s)||ws.message?.valid===false?'无有效轮速反馈':m.stopped?'已停车':'运动中';$('baseAge').textContent=age(bs);
 const speed=ws.message?.speed_mps||[];const wheels=$('wheels');wheels.replaceChildren();
 for(let i=0;i<4;i++){const el=node('div',undefined,'wheel');el.style.setProperty('--wheel',colors[i]);const invalid=stale(ws,s.settings.feedback_timeout_s)||ws.message?.valid===false;el.append(node('div',wheelNames[i]),node('b',`${n(speed[i])}${invalid?' *':''}`),node('small',`换算目标 ${n(s.command.target[i])}`),node('small',`编码器 Δ ${ws.message?.encoder_delta?.[i]??'—'}${invalid?' · 反馈无效/过期':''}`));wheels.append(el);}
 $('geometry').textContent=`换算尺寸来源：${s.geometry_source} · 轴距 ${s.settings.wheel_base_m} m / 轮距 ${s.settings.track_width_m} m。${ws.message?.valid===false?' 注意：轮速 valid=false。':''}`;
 draw('speedChart',s.history,x=>[...x.speed,...x.target],8,'m/s');draw('yawChart',s.history,x=>[Number.isFinite(x.yaw)?x.yaw*180/Math.PI:null],1,'°');
 $('active').textContent=s.startup?`正在采集：事件 ${s.startup.id}，第 ${s.startup.stage_id??'—'} 段；已确认延迟 ${s.startup.delays_ms.map(x=>x===null?'等待':x+' ms').join(' / ')}`:'等待下一次从静止开始的运动。若中途打开页面，当前运动不会被判断为正常同步。';
 $('thresholds').textContent=`起转阈值 ${s.settings.start_threshold} m/s，连续 ${s.settings.confirm_samples} 个样本确认，时间差告警 ${s.settings.delay_warning_ms} ms。起步前需连续静止 ${s.settings.rest_duration_s} s；单次观察窗口 ${s.settings.startup_window_s} s。`;
 const rows=$('events');rows.replaceChildren();for(const e of s.events){const tr=node('tr');const vals=[`${e.id} / ${e.stage_id??'—'}`,labels[e.status]||e.status,...e.delays_ms.map(x=>x===null?'—':`${x} ms`),e.spread_ms===null?'—':`${e.spread_ms} ms`,e.yaw_change_deg===null?'—':`${n(e.yaw_change_deg,2)} °`,`${e.slowest||''} ${e.reason}；反馈间隔 ${e.feedback_interval_ms??'—'} ms`];vals.forEach((v,i)=>tr.append(node('td',v,i===1?(e.status==='delayed'||e.status==='not_started'?'warn':e.status==='within_resolution'?'good':'muted'):'')));rows.append(tr);}
 const us=topic(s,'/base/ultrasonic');$('ultrasound').replaceChildren(node('p',age(us),stale(us)?'bad':'muted'));if(us.message){for(const [key,value] of Object.entries(us.message)){if(key==='header')continue;const row=node('div',undefined,'sensor');row.append(node('span',key),node('b',typeof value==='number'?n(value):String(value)));$('ultrasound').append(row);}}
 $('vision').replaceChildren();for(const name of ['/base/vision_stop_state','/vision/row_observation','/vision/harvest_targets','/vision/mango_target']){const t=topic(s,name),d=node('details');d.append(node('summary',`${name} · ${age(t)}`),node('pre',JSON.stringify(t.message,null,2)));if(name==='/base/vision_stop_state')d.open=true;$('vision').append(d);}
 $('logs').replaceChildren();if(!s.logs.length)$('logs').append(node('p','未收到 WARN / ERROR（不代表所有节点都正常）','muted'));for(const l of s.logs){$('logs').append(node('div',`[${l.name||''}] ${l.msg||''}`,'log'));}
 const filter=$('filter').value.toLowerCase();const root=$('topics');const expanded=new Set([...root.querySelectorAll('details[open]')].map(el=>el.dataset.name));root.replaceChildren();
 for(const [name,t] of Object.entries(s.topics)){if(!name.toLowerCase().includes(filter))continue;const d=node('details');d.dataset.name=name;d.open=expanded.has(name);d.append(node('summary',`${name} · ${age(t)} · #${t.count}`,stale(t)?'muted':''),node('div',t.type,'topic-meta'),node('pre',JSON.stringify(t.message,null,2)));root.append(d);}
 $('recording').textContent=s.recording?`保存目录：${s.recording.directory} · telemetry.jsonl / startup_events.csv · 丢弃记录 ${s.recording.dropped}${s.recording.error?' · 写入失败：'+s.recording.error:''}`:'记录已关闭（--no-record）';
 $('nodes').textContent=`节点：${s.nodes.join(' · ')||'等待 ROS 图发现'}`;$('graph').textContent=s.graph.map(x=>x.name+'  '+x.types.join(', ')).join('\n');
}
async function poll(){try{const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(3000)});if(!response.ok)throw new Error('HTTP '+response.status);latest=await response.json();lastSuccess=Date.now();$('connection').textContent=latest.mode==='demo'?'模拟数据':'监测服务在线';$('connection').className='pill good';render(latest);}catch(e){$('connection').textContent='监测连接中断';$('connection').className='pill bad';$('banner').textContent=`页面未取得新数据；最后更新 ${lastSuccess?Math.round((Date.now()-lastSuccess)/1000)+' 秒前':'尚无'}。之前显示的读数已经过期。`;}
 setTimeout(poll,250);}
$('filter').addEventListener('input',()=>{if(latest)render(latest);});
$('export').addEventListener('click',()=>{if(!latest)return;const rows=[['event','stage','status','A_ms','B_ms','C_ms','D_ms','spread_ms','reason'],...latest.events.map(e=>[e.id,e.stage_id,e.status,...e.delays_ms,e.spread_ms,e.reason])];const csv='\ufeff'+rows.map(r=>r.map(v=>'"'+String(v??'').replaceAll('"','""')+'"').join(',')).join('\r\n');const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'})),a=node('a');a.href=url;a.download='wheel-startup-events.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
window.addEventListener('resize',()=>{if(latest)render(latest);});poll();
