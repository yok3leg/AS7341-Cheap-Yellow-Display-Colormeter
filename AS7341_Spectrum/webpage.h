// Web page served by the ESP32 access point. Single file, no external resources,
// so it works with a phone that has no internet connection.
#pragma once
#include <pgmspace.h>

const char INDEX_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Colormeter</title>
<style>
 body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif}
 header{display:flex;justify-content:space-between;align-items:center;padding:8px 12px;background:#1b1b1b}
 header b{font-size:18px} #st{font-size:13px;color:#9cf}
 #sw{display:inline-block;width:18px;height:18px;border:1px solid #666;vertical-align:middle;margin-right:6px}
 canvas{display:block;width:100%;background:#000}
 .row{display:flex;flex-wrap:wrap;gap:6px;padding:8px 10px}
 button{flex:1 1 auto;min-width:80px;padding:10px 6px;font-size:14px;border:1px solid #555;border-radius:8px;background:#2a2a2a;color:#eee}
 button.on{background:#0b4f6c;border-color:#3cf}
 button:active{background:#444}
 table{width:100%;border-collapse:collapse;font-size:13px;margin:4px 0 12px}
 td,th{padding:3px 6px;text-align:right;border-bottom:1px solid #333} th{color:#aaa;font-weight:normal}
 td:first-child,th:first-child{text-align:left}
 .dim{color:#888;font-size:12px;padding:0 12px 12px}
</style></head><body>
<header><b>AS7341 Colormeter</b><span id="st"><span id="sw"></span>connecting...</span></header>
<canvas id="c" height="260"></canvas>
<div class="row">
 <button id="bView" onclick="cycleView()">BOTH</button>
 <button id="bNorm" onclick="norm=!norm;ui()">NORM</button>
 <button id="bExtra" onclick="extra=!extra;ui()">CLR/NIR</button>
</div>
<div class="row">
 <button id="bBg" onclick="cmd(bgSet?'bg=clear':'bg=set')">SET BG</button>
 <button id="bLed" onclick="cmd('led='+(led?0:1))">LED</button>
 <button id="bHold" onclick="cmd('hold='+(hold?0:1))">RUN</button>
 <button id="bGain" onclick="cycleGain()">GAIN</button>
</div>
<div class="row">
 <button id="bRec" onclick="toggleRec()">● Record</button>
 <button onclick="snap()">+ Snapshot</button>
 <button id="bSave" onclick="saveCSV()">Save CSV (0)</button>
 <button onclick="rows=[];ui()">Clear rows</button>
</div>
<table id="tbl"></table>
<div class="dim">Values shown are gain- and time-normalised counts (counts / s / unit gain), background-subtracted when a BG is set. CSV includes raw ADC counts as well. Saved files land in the phone's Downloads folder.</div>
<script>
const WL=[415,445,480,515,555,590,630,680],LBL=['415','445','480','515','555','590','630','680','CLR','NIR'];
const GAINS=[0.5,1,2,4,8,16,32,64,128,256,512];
function wl2rgb(w){let r=0,g=0,b=0;
 if(w<440){r=-(w-440)/60;b=1}else if(w<490){g=(w-440)/50;b=1}else if(w<510){g=1;b=-(w-510)/20}
 else if(w<580){r=(w-510)/70;g=1}else if(w<645){r=1;g=-(w-645)/65}else{r=1}
 let f=1;if(w<420)f=.3+.7*(w-380)/40;const p=x=>Math.round(255*Math.pow(x*f,.8));return [p(r),p(g),p(b)];}
const COL=WL.map(w=>`rgb(${wl2rgb(w).join(',')})`).concat(['#ddd','#8c0032']);
let view='both',norm=true,extra=false,rows=[],recording=false,last=null,bgSet=false,led=false,hold=false,gainSel=-1,inited=false;
const cv=document.getElementById('c'),ctx=cv.getContext('2d');
function cycleView(){view=view=='bars'?'line':view=='line'?'both':'bars';ui()}
function cycleGain(){gainSel=gainSel>=GAINS.length-1?-1:gainSel+1;cmd('gain='+(gainSel<0?'auto':gainSel))}
function cmd(q){fetch('/cmd?'+q,{cache:'no-store'}).catch(()=>{})}
function ui(){
 const B=(id,t,on)=>{const e=document.getElementById(id);e.textContent=t;e.className=on?'on':''};
 B('bView',view.toUpperCase(),view!='bars');B('bNorm',norm?'NORM':'ABS',!norm);B('bExtra',extra?'CLR/NIR ON':'CLR/NIR OFF',extra);
 B('bBg',bgSet?'CLR BG':'SET BG',bgSet);B('bLed',led?'LED ON':'LED OFF',led);B('bHold',hold?'HOLD':'RUN',hold);
 B('bGain',gainSel<0?'AUTO':GAINS[gainSel]+'x',gainSel>=0);
 B('bRec',recording?'■ Stop':'● Record',recording);document.getElementById('bSave').textContent='Save CSV ('+rows.length+')';
 if(last)draw(last);
}
function catmull(p0,p1,p2,p3,t){const t2=t*t,t3=t2*t;return .5*((2*p1)+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t2+(-p0+3*p1-3*p2+p3)*t3)}
function niceCeil(v){if(v<=0)return 1;const p=Math.pow(10,Math.floor(Math.log10(v))),m=v/p;return (m<=1?1:m<=2?2:m<=5?5:10)*p}
function draw(d){
 const W=cv.clientWidth;if(cv.width!=W)cv.width=W;const H=cv.height;
 ctx.fillStyle='#000';ctx.fillRect(0,0,W,H);
 const n=extra?10:8,vals=d.bc.slice(0,n);
 let mx=Math.max(...vals,1e-9),scale=norm?100:niceCeil(mx*1.05);
 const v=vals.map(x=>norm?x/mx*100:x);
 const L=44,R=8,T=14,B=38,pw=W-L-R,ph=H-T-B,slot=pw/n,bw=slot*.7;
 ctx.font='11px sans-serif';ctx.textAlign='right';ctx.textBaseline='middle';
 for(let k=0;k<=4;k++){const y=T+ph-ph*k/4;ctx.strokeStyle=k?'#222':'#777';ctx.beginPath();ctx.moveTo(L,y);ctx.lineTo(W-R,y);ctx.stroke();
  ctx.fillStyle='#888';ctx.fillText(norm?(k*25)+'%':fmt(scale*k/4),L-4,y)}
 const cx=[],h=[];for(let i=0;i<n;i++){cx.push(L+slot*i+slot/2);h.push(Math.min(v[i]/scale,1)*ph)}
 if(view!='line')for(let i=0;i<n;i++){ctx.fillStyle=COL[i];ctx.fillRect(cx[i]-bw/2,T+ph-h[i],bw,h[i]);
  if(d.raw[i]>=d.fs-1){ctx.strokeStyle='#f33';ctx.strokeRect(cx[i]-bw/2,T,bw,ph)}}
 if(view!='bars'){const pts=[];for(let x=cx[0];x<=cx[7];x++){const seg=Math.min(Math.floor((x-cx[0])/slot),6),t=(x-cx[seg])/slot;
   const hh=Math.max(0,Math.min(ph,catmull(h[Math.max(seg-1,0)],h[seg],h[seg+1],h[Math.min(seg+2,7)],t)));pts.push([x,T+ph-hh,WL[seg]+t*(WL[seg+1]-WL[seg])])}
  if(view=='line'){for(const [x,y,w] of pts){const c=wl2rgb(w);ctx.fillStyle=`rgb(${c[0]*.75|0},${c[1]*.75|0},${c[2]*.75|0})`;ctx.fillRect(x,y,1,T+ph-y)}
   for(let i=8;i<n;i++){ctx.fillStyle=COL[i];ctx.fillRect(cx[i]-4,T+ph-h[i],8,h[i])}}
  ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.beginPath();pts.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.stroke();ctx.lineWidth=1}
 ctx.textAlign='center';ctx.textBaseline='top';
 for(let i=0;i<n;i++){ctx.fillStyle=COL[i];ctx.fillText(LBL[i],cx[i],T+ph+4);ctx.fillStyle='#bbb';ctx.fillText(norm?v[i].toFixed(0):fmt(v[i]),cx[i],T+ph+18)}
}
function fmt(v){return v>=10000?(v/1000).toFixed(0)+'k':v>=1000?(v/1000).toFixed(1)+'k':v>=100?v.toFixed(0):v>=10?v.toFixed(1):v.toFixed(2)}
function status(d){
 const vis=d.bc.slice(0,8),mx=Math.max(...vis,1e-9);let r=0,g=0,b=0,pk=0;
 vis.forEach((x,i)=>{const c=wl2rgb(WL[i]),w=x/mx;r+=w*c[0];g+=w*c[1];b+=w*c[2];if(x>vis[pk])pk=i});
 const m=Math.max(r,g,b,1);document.getElementById('sw').style.background=`rgb(${r/m*255|0},${g/m*255|0},${b/m*255|0})`;
 document.getElementById('st').innerHTML=`<span id="sw" style="background:rgb(${r/m*255|0},${g/m*255|0},${b/m*255|0})"></span>peak ${WL[pk]} nm · ${d.gain}x · ${d.tint} ms · ${d.hz.toFixed(1)} Hz${d.bg?' · BG':''}${d.hold?' · HOLD':''}`;
 const n=extra?10:8;let t='<tr><th>ch</th>'+LBL.slice(0,n).map(l=>'<th>'+l+'</th>').join('')+'</tr>';
 t+='<tr><td>raw</td>'+d.raw.slice(0,n).map(x=>'<td>'+x+'</td>').join('')+'</tr>';
 t+='<tr><td>value</td>'+d.bc.slice(0,n).map(x=>'<td>'+fmt(x)+'</td>').join('')+'</tr>';
 document.getElementById('tbl').innerHTML=t;
}
function row(d){return [new Date().toISOString(),d.t,d.gain,d.tint,d.bg?1:0,d.led?1:0].concat(d.raw,d.bc.map(x=>+x.toPrecision(5)))}
function toggleRec(){recording=!recording;ui()}
function snap(){if(last){rows.push(row(last));ui()}}
function saveCSV(){
 const hdr=['time_iso','device_ms','gain','tint_ms','bg_applied','led'].concat(LBL.map(l=>'raw_'+l),LBL.map(l=>'val_'+l));
 const csv=[hdr.join(',')].concat(rows.map(r=>r.join(','))).join('\n');
 const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));
 a.download='colormeter_'+new Date().toISOString().replace(/[:T]/g,'-').slice(0,19)+'.csv';document.body.appendChild(a);a.click();a.remove();
}
async function poll(){
 try{const d=await (await fetch('/data',{cache:'no-store'})).json();last=d;bgSet=d.bg;led=d.led;hold=d.hold;gainSel=d.gsel;
  if(!inited){extra=d.extra;inited=true}
  if(recording&&!hold)rows.push(row(d));draw(d);status(d);ui();
 }catch(e){document.getElementById('st').textContent='no connection';}
 setTimeout(poll,150);
}
window.addEventListener('resize',()=>last&&draw(last));ui();poll();
</script></body></html>
)rawliteral";
