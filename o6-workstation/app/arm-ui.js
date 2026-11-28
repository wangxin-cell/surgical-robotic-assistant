const byId = id => document.getElementById(id);
const JOINTS = ["肩旋", "肩升", "肘部", "腕弯", "腕旋", "夹爪"];
const INKS = ["#92ebbd", "#f7bf73", "#88bdff", "#f99ba1", "#d1a9ff", "#81dce0"];
const number = value => Number.isFinite(value) ? String(value) : "—";
const clock = t => {
  const s = Math.floor(Math.max(t, 0));
  return Math.floor(s / 60) + ":" + String(s % 60).padStart(2, "0");
};
function cell(parent, tag, text) {
  const node = document.createElement(tag);
  node.textContent = text;
  parent.append(node);
  return node;
}

export function setupArmWorkspace(api, cancelVoice) {
  let arm = "left", active = false, revision = 0;
  let episodeVersion = 0, listVersion = 0, statusVersion = 0;
  let readingStatus = false, launching = false, activeMode = null, telemetryAvailable = false;
  let launchError = "";
  let sample = null;
  let trajectory = null, end = 0, position = 0, nextPaint = 0, anim = 0, lastTick = 0;
  let episodeList = [];
  const graph = byId("traceGraph");
  const timer = byId("traceSeek");
  const legendValues = [];
  const jointValues = [];
  const jogButtons = [];

  for (const [index, name] of JOINTS.entries()) {
    const row = byId("jointRows").insertRow();
    const header = cell(row, "th", name);
    header.scope = "row";
    cell(header, "small", "ID " + (index + 1));
    jointValues.push(["position", "goal", "torque"].map(() => cell(row, "td", "—")));
    const jog = row.insertCell();
    jog.className = "jog-controls";
    for (const sign of [-1, 1]) {
      const button = cell(jog, "button", sign < 0 ? "−" : "+");
      button.className = "arm-secondary";
      button.type = "button";
      button.disabled = true;
      button.setAttribute("aria-label", name + (sign < 0 ? "减小" : "增大") + "位置");
      button.addEventListener("click", () => act("jog", index + 1, sign * Number(byId("armStep").value)));
      jogButtons.push(button);
    }
    const item = cell(byId("traceLegend"), "div", "");
    item.className = "trace-key key-" + index;
    cell(item, "span", name);
    legendValues.push(cell(item, "strong", "—"));
  }
  function side() { return arm === "left" ? "左臂" : "右臂"; }
  function refreshButtons() {
    const pending = launching || !telemetryAvailable;
    byId("openHardware").disabled = pending || activeMode !== null;
    byId("openSimulation").disabled = pending || activeMode !== null;
    const live = sample !== null;
    const enabled = live && !pending;
    const holding = !!sample?.holding && sample.joints?.length === 6 && sample.joints.every(j => j.torque === 1);
    const supported = byId("armSupport").checked;
    const recording = !!sample?.recording;
    byId("openHardware").hidden = byId("openSimulation").hidden = !!activeMode;
    byId("closeArm").hidden = !activeMode;
    byId("closeArm").disabled = pending || !activeMode || recording;
    byId("teachControls").hidden = byId("recordTools").hidden = !live;
    byId("armHold").disabled = !enabled || !supported || sample.joints?.some(j => j.torque !== 0);
    byId("armRelease").disabled = !enabled || !supported || !sample.joints?.some(j => j.torque === 1);
    byId("armSupport").disabled = !enabled;
    byId("armStep").disabled = !enabled || !holding;
    for (const button of jogButtons) button.disabled = !enabled || !holding || sample.locked;
    byId("startRecord").disabled = !enabled || recording;
    byId("stopRecord").disabled = !enabled || !recording;
    document.querySelectorAll("[data-arm]").forEach(button => {button.disabled = launching;});
  }

  function renderStatus(data) {
    telemetryAvailable = true;
    const snap = data.telemetry;
    const closed = data.session === "closed" || snap?.state === "closed";
    const stale = data.stale && !closed;
    const live = data.session === "open" && !stale && snap?.state === "connected";
    const busy = ["open","starting"].includes(data.session) && !closed;
    sample = live ? snap : null;
    activeMode = busy ? data.mode : null;
    const simulation = data.mode === "simulation" || snap?.simulated;
    const origin = simulation ? "模拟" : "实机";
    let state = "未开启";
    if (closed) state = "示教已结束";
    else if (data.session === "error" || snap?.state === "fault") state = origin + "异常";
    else if (stale) state = "状态未更新";
    else if (live) state = origin + "已连接" + (!simulation && snap.port ? " · " + snap.port : "");
    else if (busy) state = data.session === "starting" ? "连接中" : origin + "待连接";
    const badge = byId("armState");
    badge.textContent = state;
    badge.dataset.tone = live ? "online" : (stale || state.includes("异常")) ? "warn" : "";
    const note = byId("armStatusNote");
    if (data.session === "idle") note.textContent = "点击「连接实机」，从 O6 的 USB 串口读取六个关节。";
    else note.textContent = data.message || snap?.message || state;
    if (busy && snap?.message && snap.message !== data.message) note.textContent = snap.message;
    if (stale) note.textContent = data.message || "正在等待 O6 更新六轴读数。";
    if (live && !simulation && !snap.holding && snap.joints?.some(j => j.torque === 1)) {
      note.textContent = "舵机已有保持力矩。请用独立支撑托稳臂身，在此勾选支撑确认并点「支撑后释放」。六轴力矩关闭后可启用保持。";
    }
    if (launchError) note.textContent = launchError;
    byId("jointTitle").textContent = live ? (simulation ? "模拟关节读数" : "实机关节读数") : "关节读数";
    const stamp = new Date(snap?.updated_at);
    byId("jointTime").textContent = live && Number.isFinite(stamp.valueOf())
      ? stamp.toLocaleTimeString("zh-CN", {hour12:false}) + " · 格" : "";
    const joints = live ? snap.joints || [] : [];
    JOINTS.forEach((_, i) => {
      const j = joints.find(x => x.id === i + 1);
      const [pos, goal, torque] = jointValues[i];
      pos.textContent = number(j?.position);
      goal.textContent = j?.torque === 1 ? number(j.goal) : "—";
      torque.textContent = j?.torque === 0 ? "关闭" : j?.torque === 1 ? "保持" : "—";
      pos.title = Number.isFinite(j?.min) && Number.isFinite(j?.max)
        ? "舵机限位 " + j.min + "–" + j.max + " 格" : "";
    });
    byId("armFlags").textContent = [snap?.locked && "点动已锁定", live && snap?.recording && "正在记录",
      live && snap?.recording && snap?.phase].filter(Boolean).join(" · ");
    refreshButtons();
  }
  async function status() {
    if (!active || document.hidden || readingStatus || launching) return;
    const token = ++statusVersion;
    readingStatus = true;
    try {
      const result = await api("/api/arm/status?arm=" + arm);
      if (token === statusVersion) renderStatus(result);
    } catch (e) {
      if (token === statusVersion) {
        renderStatus({session:"error",message:"读取示教状态失败 · " + e.message});
        byId("armState").textContent = "服务不可用";
        telemetryAvailable = false; refreshButtons();
      }
    } finally {
      if (token === statusVersion) readingStatus = false;
    }
  }
  async function open(mode) {
    const button = byId(mode === "hardware" ? "openHardware" : "openSimulation");
    if (button.disabled) return;
    const rev = revision;
    launchError = "";
    byId("armSupport").checked = false;
    statusVersion++; readingStatus = false; launching = true; refreshButtons();
    byId("armStatusNote").textContent = mode === "simulation" ? "正在准备模拟示教…" : "正在读取 O6 上的" + side() + "控制板…";
    try {
      const data = await api("/api/arm/open", {method:"POST", headers:{"Content-Type":"application/json"},
        body:JSON.stringify({arm,mode})});
      if (rev === revision) renderStatus(data);
    } catch (e) {
      if (rev === revision) {
        launchError = e.message;
        byId("armStatusNote").textContent = launchError;
      }
    } finally {
      if (rev === revision) {launching = false; refreshButtons();}
    }
  }
  async function act(operation, joint, delta) {
    if (launching || !activeMode) return;
    const rev = revision, mode = activeMode;
    const args = {arm, mode, operation, joint, delta, supported:byId("armSupport").checked};
    launchError = "";
    byId("armSupport").checked = false;
    launching = true; ++statusVersion; readingStatus = false; refreshButtons();
    try {
      const data = await api("/api/arm/action", {method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(args)});
      if (rev === revision) {
        renderStatus(data);
        if (operation === "record_stop") list();
      }
    } catch (e) {
      if (rev === revision) {
        launchError = e.message;
        sample = null;
        byId("armStatusNote").textContent = launchError;
      }
    } finally {
      if (rev === revision) { launching = false; refreshButtons(); status(); }
    }
  }
  function stop() {
    if (anim) cancelAnimationFrame(anim);
    anim = 0; byId("tracePlay").textContent = "播放预览";
  }
  function clearTrajectory(message) {
    stop(); trajectory = null; end = position = 0;
    byId("traceEmpty").hidden = false; byId("traceEmpty").textContent = message;
    byId("traceBody").hidden = true; byId("traceMeta").textContent = "";
    byId("tracePhases").replaceChildren();
  }
  function frameAt(t) {
    const frames = trajectory.frames;
    let low = 0, high = frames.length - 1;
    while (low < high) {
      const mid = Math.ceil((low + high) / 2);
      if (frames[mid].t <= t) low = mid; else high = mid - 1;
    }
    return frames[low];
  }
  function draw() {
    if (!trajectory || !active || !graph.clientWidth) return;
    const ctx = graph.getContext("2d");
    if (!ctx) return;
    const w = graph.clientWidth, h = graph.clientHeight, scale = Math.min(devicePixelRatio || 1, 2);
    if (graph.width !== Math.round(w*scale) || graph.height !== Math.round(h*scale)) {
      graph.width = Math.round(w*scale); graph.height = Math.round(h*scale);
    }
    ctx.setTransform(scale, 0, 0, scale, 0, 0);
    ctx.clearRect(0, 0, w, h); ctx.font = "11px monospace";
    const left = 34, right = w - 7, top = 8, bottom = h - 9;
    const x = t => left + (right-left)*t / Math.max(end, .001);
    const y = v => bottom - (bottom-top)*v / 4096;
    for (const n of [0,1024,2048,3072,4096]) {
      ctx.beginPath(); ctx.strokeStyle="#384c45"; ctx.lineWidth=1;
      ctx.moveTo(left,y(n)); ctx.lineTo(right,y(n)); ctx.stroke();
      ctx.fillStyle="#b9cac2"; ctx.fillText(n,0,y(n)+3);
    }
    for (let i=0;i<JOINTS.length;i++) {
      ctx.beginPath(); ctx.strokeStyle=INKS[i]; ctx.lineWidth=1.7;
      trajectory.frames.forEach((f,index) => {
        if (index) ctx.lineTo(x(f.t),y(f.positions[i]));
        else ctx.moveTo(x(f.t),y(f.positions[i]));
      });
      ctx.stroke();
    }
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5; ctx.beginPath();
    ctx.moveTo(x(position),top); ctx.lineTo(x(position),bottom); ctx.stroke();
  }
  function seek(t) {
    if (!trajectory) return;
    position = Math.min(end, Math.max(0, t)); timer.value = position;
    const f = frameAt(position);
    legendValues.forEach((node,i) => {node.textContent = number(f.positions[i]);});
    byId("plotTime").textContent = "取样 " + f.t.toFixed(1) + " 秒";
    byId("traceClock").textContent = clock(position) + " / " + clock(end);
    timer.setAttribute("aria-valuetext", position.toFixed(1) + " 秒");
    draw();
  }
  function play() {
    if (!trajectory || end <= 0) return;
    if (anim) {stop(); return;}
    if (position >= end) seek(0);
    const initial = position;
    const start = performance.now();
    byId("tracePlay").textContent = "暂停预览"; lastTick = 0;
    function tick(ts) {
      const t = initial + (ts-start)/1000;
      if (t >= end) {seek(end); stop(); return;}
      if (ts-lastTick >= 80) {seek(t); lastTick=ts;}
      anim = requestAnimationFrame(tick);
    }
    anim = requestAnimationFrame(tick);
  }
  function loadFrames(data) {
    const meta = [data.simulated ? "模拟记录" : "历史实测", data.action_count+" 次动作",
      data.sample_count+" 帧读数", "时长 "+clock(data.duration_s), data.complete ? "" : "记录未结束"];
    if (!data.frames?.length) {
      clearTrajectory("这份记录没有完整的六关节采样。");
      byId("traceMeta").textContent = meta.filter(Boolean).join(" · ");return;
    }
    trajectory = data;
    end = Math.max(data.duration_s || 0, data.frames.at(-1).t);
    byId("traceMeta").textContent = meta.filter(Boolean).join(" · ");
    byId("traceEmpty").hidden = true; byId("traceBody").hidden = false;
    timer.max = end; timer.disabled = end <= 0; byId("tracePlay").disabled = end <= 0;
    const phaseRow = byId("tracePhases"); phaseRow.replaceChildren();
    for(const phase of data.phases || []) {
      const btn = cell(phaseRow,"button", clock(phase.t) + " " + phase.label);
      btn.type = "button";btn.addEventListener("click",()=>{stop();seek(phase.t);});
    }
    seek(0);
  }
  async function loadEpisode(id) {
    const token = ++episodeVersion;
    clearTrajectory("正在读取轨迹…");
    try {
      const data = await api("/api/arm/episode?" + new URLSearchParams({arm,id}));
      if (token === episodeVersion) loadFrames(data);
    } catch (e) {
      if (token === episodeVersion) clearTrajectory("轨迹读取失败 · "+e.message);
    }
  }
  async function list() {
    const select = byId("trajectory"), token = ++listVersion;
    const prior = select.value;
    episodeVersion++; clearTrajectory("正在读取记录…"); select.disabled = true;
    try {
      const data = await api("/api/arm/episodes?arm=" + arm);
      if (token !== listVersion) return;
      episodeList = data.episodes || [];
      select.replaceChildren();
      for (const e of episodeList) select.add(new Option(e.label, e.id));
      if (!episodeList.length) {
        select.add(new Option("暂无记录","")); clearTrajectory(side()+"还没有可预览的示教记录。"); return;
      }
      const pick = episodeList.find(e=>e.id===prior) || episodeList.find(e=>e.preferred) || episodeList[0];
      select.value = pick.id; select.disabled = false;
      await loadEpisode(pick.id);
    } catch(e) {
      if (token === listVersion) {
        select.replaceChildren(new Option("无法读取","")); clearTrajectory("记录读取失败 · "+e.message);
      }
    }
  }
  function activate() {
    revision++; statusVersion++; readingStatus=false; launching=false;
    launchError=""; telemetryAvailable=false;
    byId("armSupport").checked=false;
    byId("controlTitle").textContent = side() + "控制";
    byId("trajectory").replaceChildren(new Option("正在读取…",""));
    renderStatus({session:"idle"}); telemetryAvailable=false;refreshButtons();
    byId("armState").textContent="读取中";
    document.querySelectorAll("[data-arm]").forEach(b=>b.setAttribute("aria-pressed",String(b.dataset.arm===arm)));
    status(); list();
  }
  function route() {
    const selected = location.hash === "#arms";
    active = selected;
    byId("visionPage").hidden = selected; byId("armPage").hidden = !selected;
    const here = selected ? "navArms" : "navVision", other = selected ? "navVision" : "navArms";
    byId(here).setAttribute("aria-current","page"); byId(other).removeAttribute("aria-current");
    document.title=(selected?"机械臂示教":"器具调取")+" · O6 工作台";
    if (selected) {cancelVoice();activate();} else {stop();}
  }
  byId("openHardware").addEventListener("click",()=>open("hardware"));
  byId("openSimulation").addEventListener("click",()=>open("simulation"));
  byId("armSupport").addEventListener("change",refreshButtons);
  for (const [id,operation] of Object.entries({closeArm:"close",armHold:"hold",armRelease:"release",startRecord:"record_start",stopRecord:"record_stop"})) {
    byId(id).addEventListener("click",()=>act(operation));
  }
  byId("trajectory").addEventListener("change",e=>loadEpisode(e.target.value));
  byId("reloadEpisodes").addEventListener("click",list);
  timer.addEventListener("input",()=>{stop();seek(Number(timer.value));});
  byId("tracePlay").addEventListener("click",play);
  document.querySelectorAll("[data-arm]").forEach(btn=>btn.addEventListener("click",()=>{
    if (arm !== btn.dataset.arm) {arm=btn.dataset.arm; activate();}
  }));
  new ResizeObserver(() => {
    cancelAnimationFrame(nextPaint);
    nextPaint=requestAnimationFrame(draw);
  }).observe(graph);
  document.addEventListener("visibilitychange",()=>{if(document.hidden)stop();else status();});
  window.addEventListener("hashchange",route);
  route();
  setInterval(status,1400);
}
