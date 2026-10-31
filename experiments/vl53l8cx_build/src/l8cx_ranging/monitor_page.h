// モニタリングのページ（HTML・CSS・JS を1つに）。パケットの読み方は monitor.cpp の MonitorPacket と合わせる
// 画面の幅に合わせて伸びるので、縦向きのスマホでも崩れない（段2）。つなぎ直しも最初から入れてある
#ifndef L8CX_MONITOR_PAGE_H
#define L8CX_MONITOR_PAGE_H

static const char MONITOR_PAGE[] = R"PAGE(<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nova ToF</title>
<style>
:root{--ink:#1f2933;--muted:#5f6c7b;--line:#d9dee4;--bg:#f7f8fa;--none:#e8eaee}
*{box-sizing:border-box}
body{margin:0;padding:10px;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,sans-serif}
main{display:flex;flex-wrap:wrap;gap:14px;align-items:flex-start;justify-content:center}
h1{font-size:16px;margin:0 0 8px;text-align:center}
#state{font-weight:600;padding:6px 10px;border-radius:6px;background:#fff;border:1px solid var(--line);margin:0 auto 10px;max-width:640px;text-align:center}
#state.bad{border:2px solid #b42318}
.view{flex:1 1 340px;max-width:560px;min-width:0}
.axis{color:var(--muted);font-size:12px;text-align:center}
.axisrow{display:flex;justify-content:space-between;color:var(--muted);font-size:12px}
#grid{display:grid;gap:2px;background:var(--bg);margin:3px 0}
.c{aspect-ratio:1;display:flex;flex-direction:column;align-items:center;justify-content:center;border-radius:3px;background:var(--none);overflow:hidden;min-width:0}
.c b{font-size:clamp(11px,3.1vw,17px);font-weight:600;font-variant-numeric:tabular-nums}
.c i{font-size:clamp(9px,2.2vw,11px);font-style:normal;opacity:.8}
#legend{display:flex;align-items:center;gap:6px;color:var(--muted);font-size:12px;margin-top:6px}
#ramp{flex:1;height:10px;border-radius:3px;background:linear-gradient(90deg,hsl(215 70% 25%),hsl(215 70% 85%))}
.stats{flex:1 1 300px;max-width:560px;min-width:0}
table{width:100%;border-collapse:collapse;background:#fff;margin-bottom:10px}
td,th{border-bottom:1px solid var(--line);padding:4px 7px;text-align:right;font-variant-numeric:tabular-nums}
th{font-weight:600;color:var(--muted);font-size:12px}
td:first-child,th:first-child{text-align:left;font-variant-numeric:normal}
.bad{color:#b42318;font-weight:700}
</style></head><body>
<h1>Nova ToF モニタリング</h1>
<div id="state">接続中…</div>
<main>
<section class="view">
<div class="axis">上（Nova の後ろから前を見た向き）</div>
<div id="grid"></div>
<div class="axisrow"><span>左</span><span>下</span><span>右</span></div>
<div id="legend"><span>近い</span><div id="ramp"></div><span>2000mm 以上</span></div>
<div class="axis">上の段＝距離 mm、下の段＝状態（5・9＝有効。灰色＝無効）</div>
</section>
<section class="stats">
<table><thead><tr><th>周期・読み出し</th><th>Wi-Fi 前</th><th>Wi-Fi 後</th></tr></thead><tbody id="phase"></tbody></table>
<table><tbody id="tof"></tbody></table>
<table><tbody id="sys"></tbody></table>
<table><tbody id="page"></tbody></table>
</section>
</main>
<script>
"use strict";
const PACKET_SIZE=296,PACKET_VERSION=1,CLOSE_FULL=4001;
const STALE_MS=3000,RETRY_MS=1000,FULL_RETRY_MS=5000,ACK_MS=1000,GAP_MS=2000,FRAME_STALE_MS=500;
const PS={0:"なし",1:"MIN_MODEM（既定）",2:"MAX_MODEM",255:"不明"};
const RESET={0:"不明",1:"電源投入",2:"外部ピン",3:"ソフト",4:"パニック",5:"割り込み WDT",6:"タスク WDT",7:"その他の WDT",8:"スリープ復帰",9:"ブラウンアウト",10:"SDIO"};
const $=id=>document.getElementById(id);
let ws=null,full=false,retryAt=0,lastRx=0,lastAck=0,everOpen=false;
let received=0,reconnects=0,gaps=0,maxGap=0,pageStart=Date.now();
let cells=[],gridN=0;

function ms(us){return (us/1000).toFixed(1)}
function clock(t){const s=Math.floor(t/1000);return Math.floor(s/60)+"分"+String(s%60).padStart(2,"0")+"秒"}
function rows(el,list){el.innerHTML=list.map(r=>'<tr><td>'+r[0]+'</td><td'+(r[2]?' class="bad"':'')+'>'+r[1]+'</td></tr>').join("")}
function setState(text,bad){const s=$("state");s.textContent=text;s.className=bad?"bad":""}

function makeGrid(n){
  const g=$("grid");g.style.gridTemplateColumns="repeat("+n+",1fr)";g.innerHTML="";cells=[];
  for(let i=0;i<n*n;i++){const c=document.createElement("div");c.className="c";c.innerHTML="<b></b><i></i>";g.appendChild(c);cells.push(c)}
  gridN=n;
}
function paint(c,d,st){
  const valid=st===5||st===9;
  if(valid){
    const l=25+60*Math.min(Math.max(d,0),2000)/2000;
    c.style.background="hsl(215 70% "+l+"%)";c.style.color=l<58?"#fff":"#1f2933";
    c.firstChild.textContent=d;
  }else{
    c.style.background="";c.style.color="#5f6c7b";c.firstChild.textContent="—";
  }
  c.lastChild.textContent=st;
}
function onPacket(buf){
  if(buf.byteLength!==PACKET_SIZE||new DataView(buf).getUint8(0)!==PACKET_VERSION){setState("データの形が違う（ページを読み込み直す）",true);return}
  const v=new DataView(buf),u32=o=>v.getUint32(o,true);
  const n=v.getUint8(1),flags=v.getUint8(7),frameAge=u32(16);
  const ready=u32(24),data=u32(28),bus=u32(32),stall=u32(36),tofOk=!!(flags&1);
  if(n&&n!==gridN)makeGrid(n);
  for(let i=0;i<n*n;i++)paint(cells[i],v.getInt16(104+2*i,true),v.getUint8(232+i));
  const stale=!tofOk||frameAge>FRAME_STALE_MS;
  setState(stale?"測距が止まっている（最後のフレームから "+frameAge+"ms）":"表示中",stale);
  const ph=o=>({f:u32(o),pa:u32(o+4),pm:u32(o+8),da:u32(o+12),dm:u32(o+16)});
  const b=ph(60),r=ph(80);
  $("phase").innerHTML=[["フレーム",b.f,r.f],["周期 平均 ms",ms(b.pa),ms(r.pa)],["周期 最大 ms",ms(b.pm),ms(r.pm)],["読み出し 平均 ms",ms(b.da),ms(r.da)],["読み出し 最大 ms",ms(b.dm),ms(r.dm)]]
    .map(x=>"<tr><td>"+x[0]+"</td><td>"+x[1]+"</td><td>"+x[2]+"</td></tr>").join("");
  rows($("tof"),[["フレーム（累計）",u32(20)],["失敗 ready",ready,ready>0],["失敗 data",data,data>0],["Wire1 のエラー",bus,bus>0],["測距の停止",stall,stall>0]]);
  const reason=v.getUint8(5);
  rows($("sys"),[["起動からの時間",clock(u32(12))],["リセット理由",RESET[reason]||reason,reason===9||(reason>=4&&reason<=7)],["電池",(v.getUint16(100,true)/1000).toFixed(2)+"V"],
    ["Wi-Fi 省電力",PS[v.getUint8(4)]||v.getUint8(4)],["RSSI",v.getInt8(6)+" dBm"],["IP",(flags&2)?"固定":"DHCP"],["Wi-Fi が切れた回数",u32(48),u32(48)>0],
    ["接続",v.getUint8(2)+" / "+v.getUint8(3)+" 台"],["空きメモリ 今",u32(52).toLocaleString()],["空きメモリ 最小（起動から）",u32(56).toLocaleString()],
    ["送信を飛ばした回数",u32(44)],["ほかのタスクのログの行",u32(40)]]);
}
function showPage(){
  rows($("page"),[["このページを開いてから",clock(Date.now()-pageStart)],["受信",received],["受信の間隔 最大",maxGap+" ms"],["2秒以上の途切れ",gaps,gaps>0],["つなぎ直し",reconnects,reconnects>0]]);
}
function connect(){
  full=false;
  ws=new WebSocket("ws://"+location.host+"/ws");
  ws.binaryType="arraybuffer";
  const self=ws;
  ws.onopen=()=>{if(everOpen)reconnects++;everOpen=true;lastRx=Date.now();lastAck=lastRx;setState("つながった。データを待っている",false)};
  ws.onmessage=e=>{
    const now=Date.now();
    if(received>0){const gap=now-lastRx;if(gap>maxGap)maxGap=gap;if(gap>=GAP_MS)gaps++}
    lastRx=now;received++;
    onPacket(e.data);
    // 生存の知らせ。タブが裏でも届くように、受信に合わせて送る
    if(now-lastAck>=ACK_MS&&self.readyState===1){self.send(new Uint8Array(1));lastAck=now}
  };
  ws.onclose=e=>{
    if(self!==ws)return;
    ws=null;full=(e.code===CLOSE_FULL);
    retryAt=Date.now()+(full?FULL_RETRY_MS:RETRY_MS);
    setState(full?"満員（接続の上限に達している）。空いたらつなぐ":"切れた。つなぎ直している",true);
  };
  ws.onerror=()=>{};
}
setInterval(()=>{
  const now=Date.now();
  if(ws===null){if(now>=retryAt)connect()}
  else if(ws.readyState===1&&now-lastRx>STALE_MS){const old=ws;ws=null;retryAt=now+RETRY_MS;old.close();setState("データが来ない。つなぎ直している",true)}
  showPage();
},500);
connect();
</script></body></html>)PAGE";

#endif  // L8CX_MONITOR_PAGE_H
