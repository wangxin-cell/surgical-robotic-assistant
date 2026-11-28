// Fixed locations are normalized in the full head image, including during resize.
const el = id => document.getElementById(id);
const activeStates = new Set(["homing", "checking", "running", "stopping"]);
const states = {homing:"正在对齐示教起姿", checking:"正在核对点位", running:"正在执行夹取轨迹",
  stopping:"正在停止",
  stopped:"本轮已停止", failed:"本轮未完成", completed:"轨迹已执行，请核对实物"};
const homeStates = {homing:"左臂正在复位", stopping:"正在停止复位", stopped:"左臂复位已停止",
  failed:"左臂复位未完成", completed:"左臂公共复位完成"};
const sixCounts = values => Array.isArray(values) && values.length===6 && values.every(Number.isInteger);

export function setupPickUI({api, post, names, onLock, viewSlot}) {
  const dialog = el("pickDialog"), canvas = el("pickCanvas"), auto = el("pickAuto");
  let manualSlot = null, calibration = null, decision = null, job = {state:"idle"}, hidePrevious = false;
  let statusOnline = false, serviceVersion = 0, pending = "", failure = "", revision = 0, polling = false, free = false;
  let image = null, boxes = [], frameId = "", points = [], cursor = null, frameVersion = 0, saving = false;
  auto.checked = false;

  const currentSlot = () => decision?.ready ? decision.slot : manualSlot || (!hidePrevious && job.slot) || null;
  let posesLoaded = false;
  async function loadRecordedPoses() {
    const body = el("poseRows");
    try {
      const data = await api("/api/pick/origins", {signal:AbortSignal.timeout(4000)});
      if (![1,2,3,4].every(i => sixCounts(data?.[i]))) throw new Error();
      body.replaceChildren();
      for (const slot of [1,2,3,4]) {
        const row = body.insertRow();
        row.dataset.slot = slot;
        const header = document.createElement("th");
        header.scope = "row";
        row.append(header);
        for (const value of data[slot]) row.insertCell().textContent = value;
      }
      posesLoaded = true;
      render();
    } catch {
      body.rows[0].cells[0].textContent = "四条录制起姿读取失败，正在重试…";
    }
  }

  const busy = () => Boolean(pending || activeStates.has(job.state));
  const blocked = () => busy() || dialog.open;
  const label = (slot, id, name) => [name || names.get(id), slot && ("点位 "+slot)].filter(Boolean).join(" · ");
  function render() {
    const running = busy(), showJob = job.state !== "idle" && (!hidePrevious || running);
    if (posesLoaded) {
      for (const row of el("poseRows").rows) {
        const selected = Number(row.dataset.slot) === currentSlot();
        row.setAttribute("aria-current", String(selected));
        row.cells[0].textContent = (selected ? "当前 · " : "") + "点位 " + row.dataset.slot;
      }
    }
    const available = serviceVersion>=16 && statusOnline && free && !blocked();
    const ready = decision?.ready && available;
    const isHome = job.kind==="home";
    const heading = job.kind==="origin" && job.state==="completed" ? "点位 "+job.slot+" 起姿已就绪" :
      isHome ? homeStates[job.state] :
      [states[job.state],label(job.slot,job.target_id,job.label)].filter(Boolean).join(" · ");
    let title = calibration?.configured ? "等待器具查询" : "尚未标记夹取点位";
    let detail = calibration?.configured ? "选好器具后，这里会显示匹配的点位。" : "视觉夹取需先标记四个位置；手动回放可直接选择原示教点。";
    if (showJob) {
      title = heading;
      detail = job.message || "";
    } else if (decision) {
      title = decision.ready ? label(decision.slot, decision.id, decision.label) : "本次未选定夹取点位";
      detail = decision.message || "";
    }
    if (manualSlot && !showJob) {
      title = "已选择点位 "+manualSlot;
      detail = "将回放这个点位的原始示教文件。";
    }
    if (!statusOnline) detail = "夹取状态暂时无法读取，正在重连。";
    if (failure) detail = failure;
    el("pickSummary").textContent = title;
    el("pickDetail").textContent = detail;
    const slot = decision?.ready ? decision.slot : manualSlot;
    el("pickStart").textContent = slot ? "夹取点位 "+slot : "夹取";
    el("pickStart").disabled = !slot || !available;
    const resumable = serviceVersion >= 19 && !!job.resume_token && !activeStates.has(job.state) && !decision && !manualSlot;
    el("pickResume").hidden = !resumable;
    el("pickResume").disabled = !resumable || !available;
    el("pickResume").textContent = job.resume_label || "继续上次路径";
    el("pickOrigin").hidden = !decision?.ready || serviceVersion<15;
    el("pickOrigin").textContent = "仅到点位 "+(decision?.slot || "")+" 起姿";
    el("pickOrigin").disabled = !ready || (job.kind==="origin" && job.origin_ready && job.request_id===decision?.request);
    el("pickHome").disabled = serviceVersion<10 || !statusOnline || !free || blocked();
    el("pickCalibrate").textContent = calibration?.configured ? "重新标记点位" : "标记四个点位";
    el("pickCalibrate").disabled = !free || !statusOnline || blocked();
    el("pickSlot").disabled = !free || !statusOnline || blocked();
    auto.disabled = el("pickSlot").disabled || !calibration?.configured || serviceVersion<16;
    el("pickViewSlot").disabled = el("pickSlot").disabled || !el("pickSlot").value || !calibration?.configured;
    el("pickLive").hidden = !running;
    if (running) {
      el("pickLiveTitle").textContent = pending==="previewHome" ? "正在核对六轴目标" :
        pending==="startHome" ? "正在提交复位请求" :
        pending==="previewFixed" ? "正在核对固定点位" :
        pending==="startFixed" ? "正在启动固定点位回放" : heading || "正在提交夹取请求";
      el("pickLiveDetail").textContent = failure || (statusOnline ? job.message : detail) || "";
      el("pickStop").textContent = isHome || pending==="previewHome" || pending==="startHome" ? "停止复位" : "停止夹取";
      el("pickStop").disabled = !!pending || !activeStates.has(job.state) || job.state === "stopping";
      const progress = el("pickProgress");
      if (Number.isFinite(job.progress) && job.state === "running") progress.value = job.progress;
      else progress.removeAttribute("value");
    }
  }
  function sync() { onLock(blocked()); render(); }
  async function readStatus() {
    if (polling || pending || (document.hidden && !busy())) return;
    const seq = revision;
    polling = true;
    try {
      if (!statusOnline) {
        const identity = await api("/api/identity");
        if (seq !== revision) return;
        serviceVersion = identity.app==="o6-instrument-desk" && identity.host==="o6" ? identity.version : 0;
      }
      if (!posesLoaded) await loadRecordedPoses();
      const state = await api("/api/pick/status");
      if (seq !== revision) return;
      if (["home","fixed"].includes(state.kind) && state.request_id!==job.request_id) {
        decision=null; auto.checked=false; hidePrevious=false;
      }
      if (activeStates.has(state.state)) {
        hidePrevious = false;
        if (state.kind!=="origin" || decision?.request!==state.request_id) decision=null;
      }
      if (state.kind==="origin" && state.state==="completed" && state.origin_ready && !decision && !hidePrevious)
        decision={ready:true,request:state.request_id,slot:state.slot,id:state.target_id,label:state.label,message:state.message};
      if (state.kind==="origin" && decision?.request===state.request_id &&
          !activeStates.has(state.state) && !state.origin_ready) decision=null;
      job = state;
      statusOnline = true;
    } catch { if (seq === revision) statusOnline = false; }
    finally { polling = false; sync(); }
  }
  async function readCalibration() {
    try { calibration = await api("/api/pick/calibration"); }
    catch { calibration = null; }
    render();
  }
  async function start(direct) {
    if (!decision?.ready || serviceVersion<16 || busy() || !statusOnline || dialog.open) return;
    const selected = decision;
    decision = null;
    job = {state:"idle", slot:selected.slot, target_id:selected.id, label:selected.label};
    ++revision; pending = "start"; failure = ""; hidePrevious = false; sync();
    try { job = await post("/api/pick/start", {request_id:selected.request, direct}); }
    catch (error) { failure = error.message; }
    finally { pending = ""; sync(); await readStatus(); }
  }
  el("pickStart").addEventListener("click", () => decision?.ready ? start(true) : startFixed());
  el("pickResume").addEventListener("click", async () => {
    if (el("pickResume").disabled || document.hidden) return;
    const payload = {request_id:job.request_id, token:job.resume_token};
    ++revision; pending="resume"; failure=""; hidePrevious=false; sync();
    try { job = await post("/api/pick/resume", payload); }
    catch (error) { failure=error.message; }
    finally { pending=""; sync(); await readStatus(); }
  });
  el("pickOrigin").addEventListener("click",async ()=>{
    if (!decision?.ready || el("pickOrigin").disabled || document.hidden) return;
    const selected=decision;
    if (!window.confirm("仅移动到点位 "+selected.slot+" 的示教起姿并停住？请确认空爪、路径有净空、手已离开。")) return;
    ++revision; pending="origin"; failure=""; auto.checked=false; hidePrevious=false; sync();
    try { job=await post("/api/pick/origin",{request_id:selected.request,confirmed:true}); }
    catch(error) { failure=error.message; }
    finally { pending=""; sync(); await readStatus(); }
  });
  async function startFixed() {
    if (!manualSlot || el("pickStart").disabled || blocked()) return;
    const slot = manualSlot;
    ++revision; decision=null; auto.checked=false; pending="previewFixed"; failure=""; sync();
    try {
      const ticket = await post("/api/pick/fixed/preview", {slot});
      if (ticket.slot!==slot || typeof ticket.request_id!=="string" || !ticket.request_id)
        throw new Error("未取得所选示教点的回放确认。");
      if (document.hidden || !window.confirm(
        "启动左臂点位 "+slot+" 的固定轨迹？\n\n确认无刃模具已按原示教位置和朝向摆好，左夹爪空载，录制路径全程有净空，手已离开。\n\n本次以现场选择为准。点「确定」立即启动。")) return;
      if (document.hidden) return;
      manualSlot=null; el("pickSlot").value=""; hidePrevious=false; pending="startFixed"; sync();
      job=await post("/api/pick/fixed/start",{slot,request_id:ticket.request_id,confirmed:true});
    } catch(error) { failure=error.message; }
    finally { pending=""; sync(); await readStatus(); }
  }
  el("pickHome").addEventListener("click", async () => {
    if (el("pickHome").disabled || blocked()) return;
    ++revision; pending="previewHome"; failure=""; sync();
    try {
      const pose = await api("/api/pick/home", {signal:AbortSignal.timeout(5000)});
      if (!sixCounts(pose.targets) || typeof pose.request_id!=="string" || !pose.request_id)
        throw new Error("O6 未返回完整的六轴复位目标。");
      if (document.hidden || !window.confirm(
        "确认让左臂实机回到公共复位姿态？\n\nID1 至 ID6 目标（格）\n"+pose.targets.join("　")+
        "\n\n请确认夹爪空载、已有防坠支撑、复位路径有净空，手和衣袖已离开。")) return;
      decision=null; auto.checked=false; hidePrevious=false;
      job={kind:"home", state:"idle"}; pending="startHome"; sync();
      job=await post("/api/pick/home", {request_id:pose.request_id, confirmed:true});
    } catch(error) { failure=error.message; }
    finally { pending=""; sync(); await readStatus(); }
  });
  el("pickStop").addEventListener("click", async () => {
    if (pending || !activeStates.has(job.state)) return;
    ++revision; pending = "stop"; failure = ""; sync();
    try { job = await post("/api/pick/stop", {}); }
    catch (error) { failure = error.message; }
    finally { pending = ""; sync(); await readStatus(); }
  });
  function clear() {
    decision = null; manualSlot = null; failure = ""; hidePrevious = true; render();
  }
  el("pickSlot").addEventListener("change", () => {
    clear();
    manualSlot = Number(el("pickSlot").value) || null;
    render();
  });
  el("pickViewSlot").addEventListener("click",()=>viewSlot(Number(el("pickSlot").value)));
  async function useLookup(scene, ids, spoken, slot=null) {
    clear();
    if (!ids.length) return;
    const found = scene.results?.find(item => item.id === ids[0]);
    decision = {request:scene.request_id, id:ids[0], label:found?.label, slot:scene.pick?.slot,
      message:scene.pick?.message || "本次画面没有可执行的点位。",
      ready: scene.source === "live" && ids.length === 1 &&
        (slot===null ? found?.status === "found" && found.unique === true : scene.pick?.slot === slot) &&
        !!scene.request_id && scene.pick?.ready === true &&
        Number.isInteger(scene.pick.slot) && scene.pick.slot >= 1 && scene.pick.slot <= 4};
    render();
    if (slot===null && spoken && auto.checked && !document.hidden && !el("visionPage").hidden) await start(true);
  }

  function draw() {
    if (!image) return;
    const ctx = canvas.getContext("2d"), w = canvas.width, h = canvas.height;
    const radius = Math.max(8,Math.min(w / 100, h / 60));
    ctx.drawImage(image,0,0,w,h);
    ctx.lineWidth = Math.max(1,w/800);
    for (const [left,top,right,bottom] of boxes) {
      ctx.strokeStyle="#09675a"; ctx.strokeRect(left,top,right-left,bottom-top);
      const x=(left+right)/2, y=(top+bottom)/2;
      ctx.beginPath(); ctx.arc(x,y,Math.max(2,radius/4),0,2*Math.PI);
      ctx.fillStyle="white"; ctx.fill(); ctx.stroke();
    }
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.font = "bold "+Math.round(radius*1.3)+"px sans-serif";
    for (const [i,[x,y]] of points.entries()) {
      ctx.beginPath(); ctx.arc(x*w,y*h,radius,0,2*Math.PI);
      ctx.fillStyle = "#07594d"; ctx.fill(); ctx.strokeStyle = "white";
      ctx.lineWidth = Math.max(1,radius/8); ctx.stroke();
      ctx.fillStyle = "white"; ctx.fillText(String(i+1),x*w,y*h);
    }
    if (cursor && points.length < 4) {
      const [x,y] = cursor; ctx.beginPath();
      ctx.moveTo(x*w-radius,y*h); ctx.lineTo(x*w+radius,y*h);
      ctx.moveTo(x*w,y*h-radius); ctx.lineTo(x*w,y*h+radius);
      ctx.strokeStyle = "white"; ctx.lineWidth = 4; ctx.stroke();
      ctx.strokeStyle = "#123c30"; ctx.lineWidth = 2; ctx.stroke();
    }
  }
  function markState(message) {
    draw();
    el("pickUndo").disabled = !points.length || saving;
    el("pickSave").disabled = points.length !== 4 || saving;
    el("pickClose").disabled = saving;
    el("pickCalNote").textContent = message || (points.length === 4 ?
      "请核对 1 至 4 号点，然后保存。" : "请点击点位 "+(points.length+1)+" 的器具框中心。");
  }
  function addPoint(x,y) {
    if (!image || saving || points.length >= 4) return;
    if (points.length && x <= points[points.length-1][0]) {
      markState("下一点应在上一点的右侧，可撤回上一点。"); return;
    }
    points.push([x,y]); cursor=null; markState();
  }
  canvas.addEventListener("click", e => {
    const r = canvas.getBoundingClientRect();
    addPoint((e.clientX-r.left)/r.width,(e.clientY-r.top)/r.height);
  });
  canvas.addEventListener("keydown", e => {
    if (!image || saving) return;
    if (e.key === "Enter" && cursor) { e.preventDefault(); addPoint(...cursor); return; }
    const vector = {ArrowUp:[0,-1],ArrowDown:[0,1],ArrowLeft:[-1,0],ArrowRight:[1,0]}[e.key];
    if (!vector || points.length === 4) return;
    e.preventDefault(); cursor ??= [points.length ? points[points.length-1][0] : 0.1, 0.5];
    cursor = cursor.map((v,i)=>Math.max(0, Math.min(1, v+(e.shiftKey ? .04 : .005)*vector[i])));
    draw();
  });
  el("pickUndo").addEventListener("click",()=>{points.pop();cursor=null;markState();});
  el("pickCalibrate").addEventListener("click", async () => {
    if (blocked() || !free) return;
    const seq = ++frameVersion;
    clear(); auto.checked=false; frameId=""; points=[]; cursor=null; image=null; boxes=[];
    el("pickFrame").hidden = true; dialog.showModal(); sync(); markState("正在拍摄头部相机原图…");
    try {
      const scene = await post("/api/lookup", {ids:[]});
      if (!scene.request_id || scene.source !== "live" || scene.error || !scene.views?.head)
        throw new Error(scene.error || "这次没有取得头部相机原图。请检查相机后重新标记。");
      const raw = new Image(); raw.src = scene.views.head; await raw.decode();
      if (!dialog.open || seq !== frameVersion) return;
      frameId = scene.request_id; image = raw;
      boxes = (scene.objects || []).map(obj=>obj.box).filter(box=>
        Array.isArray(box) && box.length===4 && box.every(Number.isFinite));
      canvas.width = raw.naturalWidth; canvas.height = raw.naturalHeight;
      el("pickFrame").hidden = false; markState();
    } catch (error) { if (dialog.open && seq === frameVersion) markState(error.message); }
  });
  el("pickSave").addEventListener("click", async () => {
    if (points.length !== 4 || !frameId || saving) return;
    saving = true; markState("正在保存点位…");
    try {
      calibration = await post("/api/pick/calibration",{request_id:frameId,points});
      failure = "四个点位已保存。请说出器具名称或手动查找。"; dialog.close();
    } catch (error) { el("pickCalNote").textContent = error.message; }
    finally { saving=false; markState(el("pickCalNote").textContent); sync(); }
  });
  el("pickClose").addEventListener("click",()=>dialog.close());
  dialog.addEventListener("cancel",e=>{if(saving)e.preventDefault();});
  dialog.addEventListener("close",()=>{++frameVersion; image=null; frameId=""; sync();});
  window.addEventListener("pagehide",()=>{auto.checked=false;});
  document.addEventListener("visibilitychange",()=>{
    if (!document.hidden) { readStatus(); if (!dialog.open) readCalibration(); }
  });
  readStatus(); readCalibration();
  watchOrigins(api, currentSlot);
  setInterval(readStatus,1200);
  return {clear, useLookup, setFree(value){free=value; render();}};
}

// Read only the open teaching window's telemetry; the browser never acquires the bus.
function watchOrigins(api, currentSlot) {
  const panel = el("originPanel"), selector = el("originSlot");
  const joints = ["肩旋","肩升","肘部","腕弯","腕旋","夹爪"];
  const cells = joints.map((name,i)=>{
    const row = el("originBody").insertRow();
    const header = document.createElement("th"); header.scope="row";
    header.textContent="ID"+(i+1)+" "+name; row.append(header);
    return Array.from({length:4},()=>row.insertCell());
  });
  let origins = null, home = null, status = null, timer = null, fetching = false;
  function display() {
    const isHome = selector.value==="home";
    const slot = Number(selector.value) || currentSlot();
    const start = isHome ? home?.targets : origins?.[slot];
    const snapshot = status?.telemetry;
    const live = status?.mode==="hardware" && status.session==="open" && !status.stale &&
      snapshot?.state==="connected" && snapshot.simulated===false;
    const sample = live && Array.isArray(snapshot.joints) ? snapshot.joints : [];
    const all = joints.map((_,i)=>sample.find(j=>j.id===i+1));
    const source = isHome ? "公共复位" : slot ? "点位 "+slot+" 起姿" : "待选择点位";
    el("originSource").textContent=source+(live ? " · 左臂实机" : "");
    const notes=[];
    if (!slot && !isHome) notes.push("选择器具或点位后显示对应的六轴目标。");
    else if (!start) notes.push("正在读取"+source+"目标。");
    if (isHome && Number.isInteger(home?.tolerance)) {
      let policy = "公共复位允许每轴偏差 ±"+home.tolerance+" 格";
      if (Number.isInteger(home.max_attempts) && home.max_attempts>1)
        policy += "，可自动重试，总计最多 "+home.max_attempts+" 轮";
      notes.push(policy+"。");
    }
    if (!live) notes.push(status?.mode==="simulation" ? "当前为模拟示教，等待左臂实机读数。" :
      status?.stale ? "实机读数已过期，正在等待更新。" : "在「机械臂示教」选择左臂，点击「连接实机」可读取当前位置。");
    else if (all.every(j=>j?.torque===0)) notes.push("六轴均已卸力，无需再释放。");
    else if (all.every(j=>j?.torque===1)) notes.push("六轴保持中。");
    else if (all.every(j=>j?.torque===0 || j?.torque===1)) notes.push("部分关节保持中。");
    const outside=[], nearLimit=[];
    all.forEach((joint,i)=>{
      const [actual,reference,delta,range]=cells[i];
      const pos=joint?.position, desired=start?.[i];
      const valid=Number.isInteger(pos), known=Number.isInteger(desired);
      actual.textContent=valid ? String(pos) : "—";
      reference.textContent=known ? String(desired) : "—";
      const diff=valid && known ? pos-desired : null;
      delta.textContent=diff===null ? "—" : (diff>0?"+":"")+diff;
      const limits=Number.isInteger(joint?.min) && Number.isInteger(joint?.max);
      const margin=valid && limits ? Math.min(pos-joint.min,joint.max-pos) : null;
      const out=margin!==null && margin<0, near=margin!==null && !out && margin<24;
      const warning=out ? "超限 · " : near ? "余量 "+margin+" 格 · " : "";
      range.textContent=limits ? warning+joint.min+"–"+joint.max : "—";
      range.dataset.alert=actual.dataset.alert=String(out || near);
      if (out) outside.push("ID"+(i+1));
      if (near) nearLimit.push("ID"+(i+1));
    });
    if (outside.length) notes.push(outside.join("、")+" 当前超出硬限。");
    if (nearLimit.length) notes.push(nearLimit.join("、")+" 距硬限不足 24 格。");
    el("originNote").textContent=notes.join(" ");
  }
  async function refresh() {
    if (!panel.open || document.hidden || el("visionPage").hidden || fetching) return;
    fetching=true;
    try {
      const reads = await Promise.allSettled([
        home || api("/api/pick/home",{signal:AbortSignal.timeout(4000)}),
        origins || api("/api/pick/origins",{signal:AbortSignal.timeout(4000)}),
        api("/api/arm/status?arm=left",{signal:AbortSignal.timeout(4000)})
      ]);
      const config=reads[0].value, data=reads[1].value;
      if (sixCounts(config?.targets))
        home={targets:config.targets, tolerance:config.tolerance, max_attempts:config.max_attempts};
      if ([1,2,3,4].every(i=>sixCounts(data?.[i]))) origins=data;
      status=reads[2].value || null;
      if (panel.open) display();
    } finally { fetching=false; }
  }
  panel.addEventListener("toggle",()=>{
    clearInterval(timer); status=null; origins=null; home=null;
    if (panel.open) {display(); refresh(); timer=setInterval(refresh,1100);}
  });
  selector.addEventListener("change",display);
  el("pickSlot").addEventListener("change",()=>{if(panel.open)display();});
}
