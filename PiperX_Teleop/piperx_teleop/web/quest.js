// Quest supplies controller poses only. RGB and recording status are shown on the PC.
const statusText = document.querySelector("#status");
const enter = document.querySelector("#enter");
let ws, session, reference, serverTime = 0, lastAck = 0;
let eventSequence = 0, pendingEvents = [], previousButtons = {}, lastNotice = null;
let hapticQueue = [], hapticPlaying = false, hapticEpoch = 0;
const hapticPatterns = {
  record_started:[1,80,.5], saving:[1,160,.25], saved:[2,80,.5],
  rejected:[2,140,.25], fault:[3,250,1], home_started:[1,180,.5],
  paused:[2,120,.3],
  home_done:[2,80,.5], home_cancelled:[1,180,.25], home_saved:[2,80,.5],
};

function queueHaptic(kind) {
  if (!hapticPatterns[kind]) return;
  if (kind === "fault") { hapticQueue = []; hapticEpoch++; }
  if (hapticQueue.length < 16) hapticQueue.push(kind);
  if (!hapticPlaying) playHaptics();
}

async function playHaptics() {
  hapticPlaying = true;
  try {
    while (hapticQueue.length && session) {
      const [count,duration,intensity] = hapticPatterns[hapticQueue.shift()];
      const epoch = hapticEpoch;
      for (let i=0; i<count && epoch===hapticEpoch && session; i++) {
        for (const source of session.inputSources) {
          const gamepad = source.gamepad;
          try {
            const pulse = gamepad?.hapticActuators?.[0]?.pulse;
            const result = pulse
              ? pulse.call(gamepad.hapticActuators[0], intensity, duration)
              : gamepad?.vibrationActuator?.playEffect("dual-rumble", {
                duration, strongMagnitude:intensity, weakMagnitude:intensity});
            Promise.resolve(result).catch(() => {});
          } catch { /* Unsupported haptics must never interrupt controller input. */ }
        }
        await new Promise(resolve => setTimeout(resolve, duration+80));
      }
    }
  } finally { hapticPlaying = false; }
}

function handState(frame, source) {
  const buttons = source?.gamepad?.mapping === "xr-standard" ? source.gamepad.buttons : [];
  // Button diagnosis stays live when pose tracking is missing; tracked=false still blocks motion.
  const blank = reason => ({tracked:false, grip:(buttons[1]?.value || 0) > 0.7,
    trigger:buttons[0]?.value || 0, position:[0,0,0],
    quaternion:[0,0,0,1],tracking_reason:reason});
  if (!source) return blank("controller_missing");
  if (!source.gripSpace) return blank("grip_space_missing");
  if (source.gamepad?.mapping !== "xr-standard") return blank("gamepad_mapping");
  const pose = frame.getPose(source.gripSpace, reference);
  if (!pose) return blank("pose_missing");
  if (pose.emulatedPosition) return blank("position_emulated");
  const p = pose.transform.position, q = pose.transform.orientation;
  return {tracked:true, tracking_reason:"", grip:(buttons[1]?.value || 0) > 0.7,
    trigger:buttons[0]?.value || 0, position:[p.x,p.y,p.z], quaternion:[q.x,q.y,q.z,q.w]};
}

async function connect() {
  eventSequence = 0; pendingEvents = []; previousButtons = {}; lastNotice = null;
  ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  await new Promise((resolve,reject) => {
    const timeout = setTimeout(() => reject(Error("连接确认超时")), 3000);
    ws.onerror = () => { clearTimeout(timeout); reject(Error("连接失败，请检查电脑程序和地址")); };
    ws.onclose = () => {
      clearTimeout(timeout);
      pendingEvents = []; hapticQueue = []; hapticEpoch++;
      statusText.textContent = "连接已关闭，请查看电脑窗口。";
      session?.end();
      reject(Error("连接已关闭"));
    };
    ws.onmessage = event => {
      const data = JSON.parse(event.data);
      if (data.type === "status") {
        clearTimeout(timeout);
        lastAck = performance.now();
        serverTime = data.server_time_s;
        pendingEvents = pendingEvents.filter(event => event.id > (data.event_ack || 0));
        const notices = data.notifications || [];
        if (lastNotice === null) lastNotice = notices.at(-1)?.id || 0;
        for (const notice of notices) {
          if (notice.id <= lastNotice) continue;
          lastNotice = notice.id;
          if (data.haptics) queueHaptic(notice.kind);
        }
        statusText.textContent = `${data.mode} | ${data.saving ? "正在保存" : data.recording ? "录制中" : "未录制"}\n${data.message}`;
        resolve();
      }
    };
  });
}

enter.onclick = async () => {
  enter.disabled = true;
  try {
    if (!isSecureContext || !navigator.xr) throw Error("请使用 Quest 浏览器：Wi-Fi 用 HTTPS，USB 用 localhost。");
    // WebXR still needs an active immersive session to provide 6-DoF controllers.
    session = await navigator.xr.requestSession("immersive-vr", {requiredFeatures:["local-floor"]});
    const gl = document.querySelector("#xr").getContext("webgl", {xrCompatible:true});
    await gl.makeXRCompatible();
    session.updateRenderState({baseLayer:new XRWebGLLayer(session,gl)});
    reference = await session.requestReferenceSpace("local-floor");
    reference.addEventListener("reset", () => session?.end());
    session.addEventListener("end", () => { session=null; ws?.close(); enter.disabled=false; });
    await connect();
    let lastSent = 0;
    function tick(now,frame) {
      if (!session) return;
      session.requestAnimationFrame(tick);
      const sources = Object.fromEntries([...session.inputSources].map(x=>[x.handedness,x]));
      const left = handState(frame,sources.left), right = handState(frame,sources.right);
      const button = (side,index)=>Boolean(sources[side]?.gamepad?.buttons[index]?.pressed);
      const buttons = {record_start:button("right",5),record_stop:button("right",4),
        reset:button("left",5),set_home:button("left",4)};
      // Capture every XR-frame transition, including taps between network sends.
      for (const [name,down] of Object.entries(buttons)) {
        if (down !== (previousButtons[name] || false)) {
          pendingEvents.push({id:++eventSequence,name,down,created:performance.now()});
        }
      }
      previousButtons = buttons;
      if (pendingEvents.length > 64) { ws.close(); return; }
      // Do not bypass focus/tracking checks when the headset sleeps or loses tracking.
      const viewer = Boolean(frame.getViewerPose(reference));
      const active = session.visibilityState === "visible" && viewer;
      if (ws.readyState === WebSocket.OPEN && ws.bufferedAmount === 0 && now-lastSent >= 15) {
        const fresh = performance.now()-lastAck < 180;
        const inactive_reason = !fresh ? "server_ack_stale" : session.visibilityState === "hidden"
          ? "session_hidden" : session.visibilityState !== "visible" ? "session_blurred"
          : !viewer ? "viewer_pose_missing" : "";
        ws.send(JSON.stringify({type:"input",active:active&&fresh,age_ms:Math.max(0,performance.now()-now),
          inactive_reason,
          server_time_s:serverTime,left,right,
          buttons,events:pendingEvents.map(({id,name,down,created}) => ({id,name,down,
            age_ms:Math.max(0,performance.now()-created)}))}));
        lastSent = now;
      }
      // Submit a blank XR frame; no RGB is fetched, encoded or uploaded to the headset.
      gl.bindFramebuffer(gl.FRAMEBUFFER,session.renderState.baseLayer.framebuffer);
      gl.clearColor(0,0,0,1);
      gl.clear(gl.COLOR_BUFFER_BIT);
    }
    session.requestAnimationFrame(tick);
  } catch(error) {
    ws?.close();
    await session?.end();
    statusText.textContent = String(error);
    enter.disabled = false;
  }
};
