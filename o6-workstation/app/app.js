import {VoiceInput} from "./voice-input.js";
import {setupArmWorkspace} from "./arm-ui.js";

const el = name => document.getElementById(name);
const names = new Map();
const outcomes = {found:"已找到",absent:"未找到",uncertain:"待确认",occluded:"器具受遮挡",unavailable:"相机不可用"};
const cameras = {head:"头部",right_wrist:"右腕",left_wrist:"左腕"};
const records = [];
let phase = "idle", scene = null, role = "head", lastIds = [], activeId = "", nextCapture = false;
let generation = 0, releaseOnSpace = false, initialized = false;
let pickUI = null, pickLocked = false;
let lastSlot = null;
const storageKey = "o6-workstation-microphone";
let savedDevice = null;
try { savedDevice = localStorage.getItem(storageKey); } catch {}
const mic = new VoiceInput({
  deviceId: savedDevice || "",
  automatic: savedDevice === null,
  onLimit: () => finish(),
  onLevel: level => { el("micLevel").value = level; }
});
function rememberDevice(value) {
  mic.automatic = value === "__auto__";
  mic.deviceId = mic.automatic ? "" : value;
  try {
    if (mic.automatic) localStorage.removeItem(storageKey);
    else localStorage.setItem(storageKey, value);
  } catch {}
}
let deviceListGeneration = 0;
async function listMics() {
  const generation = ++deviceListGeneration;
  const devices = await navigator.mediaDevices?.enumerateDevices().catch(() => []) || [];
  if (generation !== deviceListGeneration) return;
  const inputs = devices.filter(d => d.kind === "audioinput" && d.deviceId);
  const menu = el("micDevice");
  const currentDefault = inputs.find(d => d.deviceId === "default");
  menu.replaceChildren(new Option("自动选择麦克风", "__auto__"),
    new Option(currentDefault?.label || "系统默认麦克风", ""));
  for (const [index, d] of inputs.entries()) {
    if (d.deviceId !== "default") menu.add(new Option(d.label || "麦克风 " + (index+1), d.deviceId));
  }
  if (mic.deviceId && !inputs.some(d => d.deviceId === mic.deviceId)) {
    if (inputs.some(d => d.label)) rememberDevice("__auto__");
    else menu.add(new Option("上次使用的麦克风", mic.deviceId));
  }
  if (!inputs.some(d => d.label)) menu.add(new Option("授权并列出麦克风…", "__permission__"));
  menu.value = mic.automatic ? "__auto__" : mic.deviceId;
}

async function api(path, params = {}) {
  const response = await fetch(path,{cache:"no-store",...params});
  const result = await response.json();
  if (!response.ok) throw new Error(result.message || "请求未完成");
  return result;
}
const post = (path,data) => api(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(data)});
function statusText(id,text,ok) {
  const n=el(id); n.className="connection "+(ok?"ok":"warn");
  const dot=document.createElement("i"); n.replaceChildren(dot,document.createTextNode(text));
}
function feedback(stage,title,detail,tone="",heard="") {
  el("stage").textContent=stage; el("headline").textContent=title;
  el("detail").textContent=detail; el("outcomes").replaceChildren();
  el("heard").hidden=!heard;
  el("heard").textContent=heard ? "原始转写「"+heard+"」" : "";
  el("headline").parentElement.dataset.tone=tone;
  const current = Boolean(activeId && scene?.request_id === activeId && lastIds.length);
  const searching = phase === "looking" && lastIds.length > 0;
  el("armQuery").hidden = !(current || searching);
  if (current || searching) {
    const photo = el("armQueryPhoto");
    photo.hidden = !current || !scene?.image;
    if (!photo.hidden) photo.src = scene.image;
    else photo.removeAttribute("src");
    const labels = current ? (scene.results || []).map(r => r.label + " · " + (outcomes[r.status] || "待确认")) : [];
    el("armQueryOutcome").textContent = labels.length > 1 ? labels.join("　") : title;
    el("armQueryDetail").textContent = current ? "采集于 " + localTime(scene.captured_at) + "　" + detail
      : lastIds.map(id => names.get(id)).join("、") + "　" + detail;
  }
}
function setPhase(value) {
  phase=value;
  const pending=["decoding","looking"].includes(value);
  el("speak").disabled=pending || value==="starting" || !initialized || pickLocked;
  el("armSpeak").disabled=value!=="idle" || !initialized || pickLocked;
  el("micDevice").disabled=value!=="idle";
  el("micMonitor").hidden=value!=="recording";
  if (value !== "recording") el("micLevel").value=0;
  el("speak").classList.toggle("recording",value==="recording");
  el("speak").querySelector("span").textContent=({starting:"等待麦克风…",recording:"结束并识别",decoding:"正在识别语音…",looking:"正在核对画面…"})[value] || "开始说话";
  el("target").disabled=value!=="idle" || !initialized || pickLocked;
  el("refresh").disabled=value!=="idle" || !initialized || pickLocked;
  pickUI?.setFree(value==="idle" && initialized);
}
function localTime(stamp) {
  const date=new Date(stamp);
  if (!stamp || !Number.isFinite(date.valueOf())) return "时间未知";
  return date.toLocaleString("zh-CN",{month:"2-digit",day:"2-digit",hour:"2-digit",minute:"2-digit",second:"2-digit",hour12:false});
}
function showExposure(hasFrame) {
  const note = el("exposureNote");
  const reading = hasFrame ? scene?.exposure?.[role] : null;
  note.hidden = !reading?.status;
  if (note.hidden) {
    note.textContent = "";
    return;
  }
  const source = scene.source === "live" && !nextCapture ? "本次" : "上次";
  const fallback = {
    balanced: "亮度已合适",
    adjusted: "已自动调整",
    limited: "仍未达到目标亮度",
    unavailable: "未取得自动曝光结果",
    fallback: "已采用备用曝光设置"
  };
  const parts = [source + cameras[role] + "曝光", reading.message || fallback[reading.status] || "状态未知"];
  if (Number.isInteger(reading.value) && reading.value > 0) {
    parts.push((reading.value / 10).toLocaleString("zh-CN",{maximumFractionDigits:1}) + " 毫秒");
  }
  note.textContent = parts.join(" · ");
  note.dataset.tone = ["limited","unavailable","fallback"].includes(reading.status) ? "warning" : "";
}
function showCamera() {
  const img=el("cameraImage"), link=el("openImage");
  const address = role==="head" ? scene?.image || scene?.views?.head : scene?.views?.[role];
  showExposure(Boolean(address));
  document.querySelectorAll("[data-role]").forEach(button=>button.setAttribute("aria-pressed",String(button.dataset.role===role)));
  img.hidden=!address; link.hidden=!address; el("viewBadge").hidden=!address; el("noImage").hidden=!!address;
  el("noImage").textContent=scene?.error?"本次画面不可用":"尚未取得"+cameras[role]+"画面";
  if (address) {
    img.src=address; img.alt=cameras[role]+"相机工作台实拍";
    link.href=scene?.views?.[role] || address;
    el("viewBadge").textContent=scene.source==="archive" || nextCapture?"上次采集":"本次采集";
  } else {
    img.removeAttribute("src"); link.removeAttribute("href");
  }
  el("timeLabel").textContent=scene?.captured_at?"拍摄于 "+localTime(scene.captured_at):"尚未取得本次画面";
  const objects=scene?.objects || [];
  el("objCount").textContent=scene?.image?"头部定位 "+objects.length+" 处":"";
  el("objects").replaceChildren();
  for(const obj of objects) {
    const tag=document.createElement("span"); tag.className="object";
    tag.textContent=obj.number+" · "+obj.label+(obj.evidence==="found"?"":"（待核）");el("objects").append(tag);
  }
}
el("cameraImage").addEventListener("error",()=>{
  showExposure(false);
  el("cameraImage").hidden=true;el("noImage").textContent="图片未能加载，请重新拍摄";el("noImage").hidden=false;
});
function addRecord(input, result, text, id="") {
  const record={time:new Date().toLocaleTimeString("zh-CN",{hour12:false}),input,result,text,id};
  records.unshift(record); records.splice(15); renderRecords();
}
function renderRecords() {
  el("historyEmpty").hidden=records.length>0;el("historyCount").textContent=records.length?records.length+" 条":"";
  el("historyList").replaceChildren();
  for(const row of records) {
    const li=document.createElement("li");li.className="record";
    for(const [tag,value] of [["time",row.time],["strong",row.input],["span",row.text]]) {
      const node=document.createElement(tag);node.textContent=value;li.append(node);
    }
    el("historyList").append(li);
  }
}
async function serviceStatus() {
  try {
    const data=await api("/api/status");
    const cameraReady=["ready","busy"].includes(data.camera);
    statusText("voiceState",data.voice?.status==="ready"?"语音模型就绪":"语音模型未就绪",data.voice?.status==="ready");
    statusText("cameraState",cameraReady?"视觉已连接":"视觉待连接",cameraReady);
    el("backend").textContent=data.voice?.status==="ready"?"O6 离线语音":"";
  } catch {
    statusText("cameraState","工作台服务离线",false);statusText("voiceState","连接待恢复",false);
  }
}
function displayResults(results, intent, heard) {
  if(results.length===1) {
    const r=results[0];
    const title=heard!==null ? outcomes[r.status] || "待确认"
      : r.status==="found"?"已找到"+r.label:r.status==="absent"?"未找到"+r.label:outcomes[r.status] || "待确认";
    const numbers=r.numbers?.length?"　画面编号 "+r.numbers.join("、"):"";
    feedback(intent,title,r.message+numbers,r.status==="found"?"positive":"warning",heard);
  } else {
    feedback(intent,"多件器具核对结果","按语音命中顺序显示。","",heard);
    for(const r of results) {
      const line=document.createElement("div");line.className="item-result";
      line.textContent=r.label+" · "+(outcomes[r.status]||"待确认")+"。"+r.message+(r.numbers?.length?"　编号 "+r.numbers.join("、"):"");
      el("outcomes").append(line);
    }
  }
}
async function inspect(ids, intent, inputLabel, heard=null, slot=null) {
  if (pickLocked) return;
  pickUI?.clear();
  lastIds=ids;lastSlot=slot;activeId="";nextCapture=true;setPhase("looking");showCamera();
  const words=ids.map(id=>names.get(id)).join("、");
  feedback(intent,slot!==null?"正在查看点位 "+slot:heard!==null?"正在核对工作台画面":ids.length?"正在查找"+words:"正在拍摄工作台","正在向 O6 获取本次工作台画面。","",heard);
  try {
    const result=slot===null ? await post("/api/lookup",{ids}) : await post("/api/pick/slot",{slot});
    scene=result;role="head";nextCapture=false;showCamera();
    if(result.error) throw new Error(result.error);
    activeId=result.request_id;
    if(ids.length) {
      displayResults(result.results,intent,heard);
      addRecord(inputLabel,"visual",result.results.map(r=>r.label+" · "+(outcomes[r.status]||r.status)).join(" / "),activeId);
      await pickUI?.useLookup(result,ids,heard!==null,slot);
    } else {feedback("画面已更新","等待语音","试说「拿直剪」或「拿组织镊」。");}
  } catch (error) {
    if(!scene?.error) {scene=null;nextCapture=false;showCamera();}
    feedback(intent,"本次视觉核对未完成",error.message,"warning",heard);
    if(ids.length)addRecord(inputLabel,"unavailable","未取得本次视觉结果");
  } finally {setPhase("idle");serviceStatus();}
}
function inspectSlot(slot) {
  if (phase!=="idle" || !initialized || !Number.isInteger(slot) || slot<1 || slot>4) return;
  const label="点位 "+slot;
  return inspect(["slot_"+slot],"按点位查找 · "+label,label+"（手动）",null,slot);
}
function micIssue(e) {
  if(e.name==="NotAllowedError")return "请检查浏览器对此页面的麦克风权限和系统录音权限，再试一次。";
  if(e.name==="NotFoundError")return "当前浏览器未找到麦克风，请连接录音设备后重试。";
  if(e.name==="NotReadableError")return "麦克风暂时无法打开，请检查设备是否被占用。";
  return e.message || "请检查麦克风后重试。";
}
async function begin(fromSpace=false, fromArms=false) {
  if(phase!=="idle" || !initialized || pickLocked || (el("visionPage").hidden && !fromArms))return;
  pickUI?.clear();
  const seq=++generation;activeId="";lastIds=[];lastSlot=null;nextCapture=true;showCamera();releaseOnSpace=fromSpace;
  setPhase("starting");feedback("麦克风","准备录音","首次使用请在本窗口允许麦克风。");
  try {
    const started=await mic.start();
    if(seq!==generation)return;
    if(!started){releaseOnSpace=false;setPhase("idle");feedback("麦克风","请再试一次","请先完成浏览器的麦克风授权，再点击开始说话。","warning");return;}
    if(mic.automatic)rememberDevice("__auto__");
    listMics();
    setPhase("recording");feedback("正在聆听","请说出所需器具",mic.label+" · "+(fromSpace?"说完松开空格。":"说完点击「结束并识别」。")+(mic.virtual?" 音量条没有变化时，请选择麦克风阵列。":""));
  } catch(e) {if(seq===generation){feedback("麦克风","录音未开始",micIssue(e),"warning");setPhase("idle");}}
}
async function finish() {
  if(phase==="starting"){cancel("录音尚未开始，请等麦克风就绪后再说。");return;}
  if(phase!=="recording")return;
  const seq=generation;
  releaseOnSpace=false;setPhase("decoding");
  feedback("语音识别","正在识别器具名称","正在用语音模型核对刚才的录音。");
  try {
    const wav=await mic.stop();if(seq!==generation)return;if(!wav)throw new Error("未取得有效录音。");
    const r=await api("/api/voice",{method:"POST",headers:{"Content-Type":"audio/wav"},body:wav});
    if(seq!==generation)return;
    const ids=[...new Set((r.matches || []).map(x=>x.id).filter(id=>names.has(id)))];
    if(!ids.length || !["matched","ambiguous","transcribed"].includes(r.status)){
      const detail=r.message || "没有识别到器具名称，请再说一次，例如「拿直剪」。";
      feedback("语音识别","没有确定器具",r.text ? "听到「"+r.text+"」。"+detail : detail,"warning");
      addRecord("语音","no_match","未识别到有效器具请求");setPhase("idle");return;
    }
    const words=ids.map(id=>names.get(id)).join("、");
    const intent="语音已识别 · "+words;
    await inspect(ids,intent,words+"（语音）",r.text || "");
  } catch(e){if(seq===generation){feedback("语音识别","语音请求未完成",micIssue(e),"warning");setPhase("idle");}}
}
function cancel(message="本次录音已取消。") {
  if(!["starting","recording","decoding"].includes(phase))return;
  ++generation;releaseOnSpace=false;mic.cancel();setPhase("idle");
  feedback("准备就绪","等待语音",message);
}
el("speak").addEventListener("click",()=>phase==="recording"?finish():begin());
el("armSpeak").addEventListener("click",()=>{
  if(phase!=="idle" || !initialized)return;
  location.hash="vision";
  begin(false,true);
  requestAnimationFrame(()=>el("speak").focus());
});
el("micDevice").addEventListener("change",async event=>{
  const value=event.target.value;
  if(value!=="__permission__"){ rememberDevice(value); return; }
  const seq=++generation;
  setPhase("starting");
  try {
    await mic.requestPermission();
    if(seq===generation)feedback("麦克风","麦克风已授权","可以选择输入设备，然后点击「开始说话」。");
  } catch(e){ if(seq===generation)feedback("麦克风","无法打开麦克风",micIssue(e),"warning"); }
  finally { await listMics(); if(seq===generation)setPhase("idle"); }
});
navigator.mediaDevices?.addEventListener("devicechange",listMics);
el("refresh").addEventListener("click",()=>{
  if(phase!=="idle")return;
  if(lastSlot!==null)inspectSlot(lastSlot);
  else inspect(lastIds,"重新拍摄",lastIds.map(id=>names.get(id)).join("、")+"（复查）");
});
el("target").addEventListener("change",event=>{const id=event.target.value;event.target.value="";if(id&&phase==="idle")inspect([id],"手动查询 · "+names.get(id),names.get(id)+"（手动）");});
document.querySelectorAll("[data-role]").forEach(button=>button.addEventListener("click",()=>{role=button.dataset.role;showCamera();}));
window.addEventListener("keydown",e=>{
  if(el("visionPage").hidden)return;
  if(e.code==="Escape"){cancel();return;}
  if(e.code!=="Space"||e.altKey||e.ctrlKey||e.metaKey||e.target.closest("select,input,textarea,[contenteditable],a,button:not(#speak)"))return;
  e.preventDefault();if(!e.repeat)begin(true);
});
window.addEventListener("keyup",e=>{if(e.code==="Space"&&releaseOnSpace){e.preventDefault();finish();}});
window.addEventListener("blur",()=>{if(phase==="recording")cancel();});
document.addEventListener("visibilitychange",()=>{if(document.hidden)cancel();else listMics();});
window.addEventListener("pagehide",()=>mic.cancel());
let receiptState="";
async function updateReceipt() {
  const id=activeId;
  if(!id || lastSlot!==null || phase!=="idle" || document.hidden)return;
  try {
    const {receipt}=await api("/api/receipt?id="+encodeURIComponent(id));
    if(id!==activeId || !receipt || phase!=="idle")return;
    const item=Object.entries(receipt.states||{}).find(([,s])=>s!=="awaiting_controller");
    if(!item)return;
    const [key,state]=item,word=names.get(key)||key,stamp=id+":"+state;
    if(receiptState===stamp)return;receiptState=stamp;
    const title=({grasp_started:"正在夹取"+word,grasped:"已夹取"+word,failed:"夹取失败 · "+word,
      awaiting_verification:"动作已结束，等待核实"+word+"是否被夹起",unconfirmed:word+"的夹取结果未确认"})[state];
    if(!title)return;
    const detail=({grasp_started:"控制器已测到夹口朝目标闭合。",grasped:"操作者已确认器具离开承托面并放回。",
      failed:"本轮控制器报告夹取失败。",awaiting_verification:"请以演示窗口中的实物观察记录为准。",unconfirmed:"本轮没有确认器具被夹起。"})[state];
    feedback("机械臂实机回执",title,detail,["failed","unconfirmed"].includes(state)?"warning":"positive");
    const row=records.find(x=>x.id===id);if(row){row.text=title;renderRecords();}
  } catch { /* Existing visual results remain visible if a controller is offline. */ }
}
async function init() {
  setPhase("idle");serviceStatus();listMics();
  try {
    const data=await api("/api/initial");
    for(const [id,label] of Object.entries(data.targets)){names.set(id,label);el("target").add(new Option(label,id));}
    scene=data.scene;showCamera();initialized=true;setPhase("idle");
  }catch(e){feedback("启动失败","工作台暂不可用",e.message,"warning");}
}
setupArmWorkspace(api, () => cancel());
import("./pick-ui.js").then(({setupPickUI})=>{
  pickUI = setupPickUI({api,post,names,viewSlot:inspectSlot,onLock(locked){
    if (locked===pickLocked) return;
    pickLocked=locked;
    if (locked && ["starting","recording","decoding"].includes(phase)) cancel();
    setPhase(phase);
  }});
  pickUI.setFree(initialized && phase==="idle");
}).catch(()=>{
  el("pickSummary").textContent="点位夹取界面未加载";
  el("pickDetail").textContent="请刷新 O6 工作台页面。";
});
init();setInterval(()=>{if(!document.hidden)serviceStatus();},9000);setInterval(updateReceipt,1400);
